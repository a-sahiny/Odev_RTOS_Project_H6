/**
  ******************************************************************************
  * @file    net_app.c
  * @brief   FreeRTOS-Plus-TCP start-up: the VNET interface over USART1
  *          (vnet_if.c), optionally the board's Ethernet port as a second one.
  ******************************************************************************
  * Address : APP_NET_IP_ADDR / APP_NET_NETMASK (192.168.50.2/24), static; no
  *           DHCP, DNS or TCP in this project (FreeRTOSIPConfig.h, D8).
  * MAC     : locally administered, derived from the 96-bit unique device ID.
  * Random  : xorshift PRNG; tohum = cihaz kimliği ^ donanım RNG'si (PLLQ
  *           48 MHz). NetApp_RandomInit, App_Init'te SysState_Init'ten
  *           (EPOCH) önce çağrılır. Laboratuvar için; güvenlik için değil.
  * Hook    : vApplicationIPNetworkEventHook_Multi sets / clears NET_READY for
  *           the VNET end-point (IP task context, docs 12) and posts
  *           NetUp / NetDown to ctrlQueue (wait 0, report 056).
  * RTOS objects: none of its own (künye: report 050).
  ******************************************************************************
  */
#include "net_app.h"

#include "main.h"
#include "task.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Routing.h"

#include "app_config.h"
#include "app_evt.h"
#include "app_stats.h"
#include "sys_state.h"
#include "vnet_if.h"

/* 1: also bring up the board's Ethernet port (RMII + LAN8742A) as a second
 * interface on its own subnet. 0: the ETH driver stays linked (it provides
 * the network buffer RAM, uxNetworkInterfaceAllocateRAMToBuffers) but is
 * never initialised (V3). */
#ifndef NET_USE_ETH
  #define NET_USE_ETH       0
#endif

static const uint8_t ucIPAddress[ 4 ] = APP_NET_IP_ADDR;
static const uint8_t ucNetMask[ 4 ]   = APP_NET_NETMASK;
static const uint8_t ucGateway[ 4 ]   = APP_NET_GATEWAY;
static const uint8_t ucDNSServer[ 4 ] = APP_NET_DNS;

/* ETH_RX_BUF_SIZE (stm32f7xx_hal_conf.h, CubeMX "Rx Buffers Length") must equal
 * the driver's RX buffer: frame size rounded up to the 32-byte cache line. */
_Static_assert(ETH_RX_BUF_SIZE == ((ipTOTAL_ETHERNET_FRAME_SIZE + 31U) & ~31U),
               "CubeMX ETH Rx Buffers Length must be 1536");

static NetworkInterface_t xVNetInterface;
static NetworkEndPoint_t xVNetEndPoint;

#if (NET_USE_ETH == 1)
/* Provided by portable/NetworkInterface/STM32/NetworkInterface.c */
extern NetworkInterface_t *pxSTM32_FillInterfaceDescriptor(BaseType_t xEMACIndex,
                                                           NetworkInterface_t *pxInterface);
static NetworkInterface_t xEthInterface;
static NetworkEndPoint_t xEthEndPoint;
static const uint8_t ucEthIPAddress[ 4 ] = { 192, 168, 1, 10 };
static const uint8_t ucEthGateway[ 4 ]   = { 192, 168, 1, 1 };
#endif

static uint32_t ulPrngState;

// RNG'den sayı alınamayan açılış sayısı (debugger'dan okunur); 0 olmalı
static volatile uint32_t rng_fail_count;

/*---------------------------------------------------------------------------*/
static uint32_t prvPrngNext(void)
{
  /* xorshift32 */
  uint32_t x = ulPrngState;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  ulPrngState = x;
  return x;
}

/* RNG'de yeni sayı hazır olana kadar bekler; hazır ve hatasızsa 1 döner. */
static int rng_wait(void)
{
    // 48 MHz'de bir sayı ~1 µs sürer; 10000 tur 216 MHz'de en az 46 µs: bol pay, açılışı uzatmaz
    uint32_t spins = 10000U;
    while (((RNG->SR & RNG_SR_DRDY) == 0U) && (spins > 0U))
    {
        --spins;
    }

    if (spins == 0U)
    {
        return 0;                                       // zaman aşımı: RNG saati yok
    }

    if ((RNG->SR & (RNG_SR_SECS | RNG_SR_CECS)) != 0U)
    {
        return 0;                                       // tohum ya da saat hatası
    }

    return 1;
}

/* Donanım RNG'sinden bir sayı okur; başarıda 1, hatada 0 döner. */
static int rng_read(uint32_t *value)
{
    /* RNG saati PLLQ: 432 MHz / 9 = 48 MHz (CK48MSEL sıfırlamada PLLQ) */
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    (void)RCC->AHB2ENR;                                 // geri okuma: saat açılmadan RNG'ye yazılmaz
    RNG->CR |= RNG_CR_RNGEN;

    /* İki sayı okunur, ikincisi kullanılır; ikisi aynıysa RNG bozuk sayılır. */
    int ok = 0;
    if (rng_wait() == 1)
    {
        const uint32_t first = RNG->DR;
        if (rng_wait() == 1)
        {
            *value = RNG->DR;
            ok = (*value != first) ? 1 : 0;
        }
    }

    /* Yalnız açılışta kullanılıyor: RNG ve saati kapatılır */
    RNG->CR &= ~RNG_CR_RNGEN;
    RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
    return ok;
}

/* DWT çevrim sayacını açar ve PRNG'yi tohumlar. App_Init'te SysState_Init'ten
 * önce bir kez çağrılır: EPOCH ilk rastgele sayıdan alınır. */
void NetApp_RandomInit(void)
{
    const uint32_t *pulUid = (const uint32_t *)UID_BASE;

    /* CYCCNT gecikme ölçümlerinin zaman tabanı (M1..M5, docs 8.1).
     * Cortex-M7: DWT yazılım kilidi açıkken DWT->CTRL yazımı yok sayılır,
     * debugger yoksa CYCCNT saymaz; anahtar kilidi açar (V6). */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* UID sabit, CYCCNT az önce sıfırlandı ve SysTick zamanlayıcıdan önce
     * çalışmıyor: açılıştan açılışa değişen tek kaynak RNG. */
    uint32_t random = 0U;
    if (rng_read(&random) == 0)
    {
        ++rng_fail_count;                               // EPOCH her açılışta aynı olabilir
        random = DWT->CYCCNT;
    }

    ulPrngState = pulUid[0] ^ (pulUid[1] << 7) ^ (pulUid[2] << 13) ^ random ^ 0x9E3779B9U;
    if (ulPrngState == 0U)
    {
        ulPrngState = 0x2545F491U;                      // xorshift 0'da takılı kalır
    }
}

/*---------------------------------------------------------------------------*/
BaseType_t NetApp_Init(void)
{
  const uint8_t *pucUid = (const uint8_t *)UID_BASE;
  uint8_t ucMAC[6];

  /* 02:xx:xx:xx:xx:xx = locally administered, unicast. */
  ucMAC[0] = 0x02;
  ucMAC[1] = 0x80;
  ucMAC[2] = 0xE1;
  ucMAC[3] = pucUid[0] ^ pucUid[4] ^ pucUid[8];
  ucMAC[4] = pucUid[1] ^ pucUid[5] ^ pucUid[9];
  ucMAC[5] = pucUid[2] ^ pucUid[6] ^ pucUid[10];

  (void)VNet_FillInterfaceDescriptor(0, &xVNetInterface);
  FreeRTOS_FillEndPoint(&xVNetInterface, &xVNetEndPoint, ucIPAddress, ucNetMask,
                        ucGateway, ucDNSServer, ucMAC);

#if (NET_USE_ETH == 1)
  /* Its own MAC (last bit flipped) and subnet: the two must not overlap. */
  ucMAC[5] ^= 0x01U;
  (void)pxSTM32_FillInterfaceDescriptor(0, &xEthInterface);
  FreeRTOS_FillEndPoint(&xEthInterface, &xEthEndPoint, ucEthIPAddress, ucNetMask,
                        ucEthGateway, ucDNSServer, ucMAC);
#endif

  return FreeRTOS_IPInit_Multi();
}

/*---------------------------------------------------------------------------*/
/* Hooks required by FreeRTOS-Plus-TCP                                       */
/*---------------------------------------------------------------------------*/
void vApplicationIPNetworkEventHook_Multi(eIPCallbackEvent_t eNetworkEvent,
                                          struct xNetworkEndPoint *pxEndPoint)
{
  /* IP task context: must not block. Only the VNET end-point drives
   * NET_READY (V8). The bit first, then the event: when ctrlTask handles
   * NetUp / NetDown, NET_READY already says the same (report 056, part 6). */
  if (pxEndPoint == &xVNetEndPoint)
  {
    const BaseType_t xUp = (eNetworkEvent == eNetworkUp) ? pdTRUE : pdFALSE;
    AppEvent_t xEvent;

    SysState_SetNetReady(xUp);

    xEvent.eEventType = (xUp == pdTRUE) ? eAppNetUpEvent : eAppNetDownEvent;
    xEvent.pvData = NULL;

    /* Wait 0 (docs 4). A full ctrlQueue loses the event, not the state:
     * NET_READY is already set or cleared (E3, E4). */
    if (AppEvt_Send(eAppTargetCtrl, &xEvent, 0U) == pdPASS)
    {
      Stats_Inc(eStatEvtNetPosted);
    }
    else
    {
      Stats_Inc(eStatEvtNetLost);
    }
  }
}

/* Report 092: no critical section. Callers: SysState_Init (before the
 * scheduler) and the IP task (UDP only: ports, IP ids); one caller at a
 * time, so the generator state needs no lock. */
BaseType_t xApplicationGetRandomNumber(uint32_t *pulNumber)
{
  *pulNumber = prvPrngNext() ^ DWT->CYCCNT;
  return pdTRUE;
}

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                            uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress,
                                            uint16_t usDestinationPort)
{
  /* TCP only (ipconfigUSE_TCP 0 here); kept so that switching TCP back on
   * does not need a new hook. */
  uint32_t ulRandom;

  (void)xApplicationGetRandomNumber(&ulRandom);
  return ulRandom ^ ulSourceAddress ^ ((uint32_t)usSourcePort << 16) ^
         ulDestinationAddress ^ usDestinationPort;
}
