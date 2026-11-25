/* MP257 hattı (USART6). meas task'ı ham telemetriyi, uartTx task'ı (Linux okurken) satırları yollar; satır parçalara
   bölünür, bekleyen telemetri bir sonraki parçadan önce gider. RX DMA'sı MP257'nin "Linux var / yok" mesajını
   ve PC'nin Linux üzerinden gelen baytlarını alır; baytlar USART1'inkilerle aynı akışa girer.
   Yalnız APP_TELEM_TARGET_MP257 derlemesinde vardır. */
#include "raw_link.h"
#include "app_config.h"

#if (APP_TELEM_TARGET == APP_TELEM_TARGET_MP257)

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "crc.h"
#include "uart_link.h"
#include <string.h>

#define FRAME_SOF      0xA5U            // çerçeve: SOF, tür, uzunluk, veri, CRC8(tür..veri); MP257'deki frame.c ile aynı
#define FRAME_TELEM    0x54U            // 'T': APP_LEN_TELEM bayt ham telemetri
#define FRAME_LINE     0x44U            // 'D': sıra baytı + satırın bir parçası
#define FRAME_OVERHEAD 4U
#define PIECE_CHARS    15U              // 460800 baud'da parça 0,43 ms: telemetri en çok bu kadar bekler
#define PIECE_FIRST    0x80U            // sıra baytında: satırın ilk parçası
#define FRAME_MAX      (FRAME_OVERHEAD + APP_LEN_TELEM)   // en uzun çerçeve telemetri

_Static_assert(1U + PIECE_CHARS <= APP_LEN_TELEM, "satır parçası tx_buf'a sığar");

#define LINUX_MSG_TYPE 0x4CU            // 'L': SOF, tür, durum, ~durum; MP257'deki f746_tx.c ile aynı
#define FRAME_COMMAND  0x43U            // 'C': sıra baytı + PC'nin baytları; CRC8'li çerçeve
#define RX_DATA_MAX    32U
#define RX_DMA_BYTES   256U             // dolması 5,6 ms
#define LINUX_TIMEOUT_MS 2500U          // MP257 mesajı saniyede bir yeniler; bir kayıp idare edilir

typedef enum
{
    TX_IDLE,
    TX_TELEM,
    TX_LINE
} TxKind;

extern UART_HandleTypeDef huart6;

// Gönderme durumu: task'lar kritik bölgede, TX bitti kesmesi kendi içinde değiştirir
static uint8_t tx_buf[FRAME_MAX];       // DMA okur; D-cache kapalı
static volatile TxKind tx_kind;         // hattaki çerçeve
static uint16_t tx_seq;                 // hattaki telemetrinin SEQ'i
static uint32_t tx_drdy;                // hattaki telemetrinin DRDY anı (DWT çevrimi)
static uint8_t telem_buf[APP_LEN_TELEM]; // sıradaki telemetri
static uint16_t telem_seq;
static uint32_t telem_drdy;
static volatile uint32_t telem_pending; // telem_buf hattı bekliyor
static const uint8_t *line;             // yollanan satır; NULL: yok
static size_t line_len;
static size_t line_pos;                 // hatta verilen karakter
static uint8_t piece_seq;               // parça sırası, 7 bit
static TaskHandle_t line_waiter;        // satırın bitmesini bekleyen task

static volatile uint32_t last_sent;     // kesme yazar, meas okur
static volatile uint32_t tx_drops;      // öncekisi hattı beklerken gelen ya da başlatılamayan çerçeve (debugger'dan okunur)
static uint8_t rx_dma[RX_DMA_BYTES];    // dairesel DMA yazar
static uint32_t rx_pos;                 // DMA arabelleğinde okunan konum
static uint8_t rx_frame[3U + RX_DATA_MAX]; // tür, uzunluk, veri, CRC8
static uint32_t rx_len;                 // toplanan bayt, SOF dahil; 0: SOF bekleniyor
static uint8_t rx_next;                 // beklenen komut sırası, 7 bit
static volatile uint32_t rx_lost;       // sırası tutmayan komut çerçevesi (debugger'dan okunur)
static volatile uint32_t linux_on;      // son mesaj: 1 Linux okuyor; kesme yazar, süre dolunca okuyan siler
static volatile uint32_t linux_tick;    // son "Linux var" anı, ms
static volatile uint32_t rx_errors;     // hat hatası (debugger'dan okunur)
static volatile uint32_t rx_bad;        // bozuk mesaj ya da çerçeve (debugger'dan okunur)

/* USART6'yı APP_RAW_BAUD hızına alır, almayı başlatır. CubeMX 230400 ile kurdu, kalan ayar ondan. */
void raw_link_init(void)
{
    huart6.Init.BaudRate = APP_RAW_BAUD;
    configASSERT(HAL_UART_Init(&huart6) == HAL_OK);

    // RX (PC7) çekme dirençli: tel takılı değilken gürültü mesaj sanılmasın
    GPIO_InitTypeDef pin = { .Pin = GPIO_PIN_7, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP,
                             .Speed = GPIO_SPEED_FREQ_VERY_HIGH, .Alternate = GPIO_AF8_USART6 };
    HAL_GPIO_Init(GPIOC, &pin);

    configASSERT(HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_dma, RX_DMA_BYTES) == HAL_OK);
}

/* tx_buf'taki 'size' baytlık veriyi çerçeveye alır ve DMA'yı başlatır. */
static void start_frame(uint8_t type, size_t size, TxKind kind)
{
    tx_buf[0] = FRAME_SOF;
    tx_buf[1] = type;
    tx_buf[2] = (uint8_t)size;
    tx_buf[3U + size] = Crc8_Compute(&tx_buf[1], 2U + size);
    tx_kind = kind;

    if (HAL_UART_Transmit_DMA(&huart6, tx_buf, (uint16_t)(FRAME_OVERHEAD + size)) != HAL_OK)
    {
        ++tx_drops;
        tx_kind = TX_IDLE;
    }
}

/* Hat boş: bekleyen telemetriyi, yoksa satırın sıradaki parçasını başlatır (kesme ya da kritik bölge içinde). */
static void start_next(void)
{
    if (telem_pending)
    {
        memcpy(&tx_buf[3], telem_buf, sizeof(telem_buf));
        tx_seq = telem_seq;
        tx_drdy = telem_drdy;
        telem_pending = 0;
        start_frame(FRAME_TELEM, sizeof(telem_buf), TX_TELEM);
        return;
    }

    tx_kind = TX_IDLE;

    if (line == NULL || line_pos == line_len)
    {
        return;
    }

    size_t n = line_len - line_pos < PIECE_CHARS ? line_len - line_pos : PIECE_CHARS;

    tx_buf[3] = (uint8_t)(piece_seq | (line_pos == 0 ? PIECE_FIRST : 0U));
    piece_seq = (uint8_t)((piece_seq + 1U) & 0x7FU);
    memcpy(&tx_buf[4], &line[line_pos], n);
    line_pos += n;
    start_frame(FRAME_LINE, 1U + n, TX_LINE);
}

/* Bir telemetri mesajını sıraya koyar, beklemez. Öncekisi hâlâ sıradaysa mesaj düşer ve sayılır. */
void raw_link_send(const uint8_t *payload, uint16_t seq, uint32_t drdy_cycles)
{
    taskENTER_CRITICAL();               // TX bitti kesmesi aynı durumu değiştirir

    if (telem_pending)
    {
        ++tx_drops;
    }
    else
    {
        memcpy(telem_buf, payload, sizeof(telem_buf));
        telem_seq = seq;
        telem_drdy = drdy_cycles;
        telem_pending = 1;

        if (tx_kind == TX_IDLE)
        {
            start_next();
        }
    }

    taskEXIT_CRITICAL();
}

/* Satırı parçalara bölüp yollar, bitmesini bekler; başarı 1. Yalnız uartTx çağırır, satır dönene kadar değişmez. */
int raw_link_send_line(const uint8_t *data, size_t size, TickType_t wait)
{
    (void)ulTaskNotifyTake(pdTRUE, 0);  // süresi dolmuş önceki satırın geç bildirimi

    taskENTER_CRITICAL();
    line = data;
    line_len = size;
    line_pos = 0;
    line_waiter = xTaskGetCurrentTaskHandle();

    if (tx_kind == TX_IDLE)
    {
        start_next();
    }

    taskEXIT_CRITICAL();

    if (ulTaskNotifyTake(pdTRUE, wait) != 0)
    {
        return 1;
    }

    // DMA takıldı: satırı bırak, hattı boşalt. Abort saat sayar, kritik bölge dışında
    taskENTER_CRITICAL();
    line = NULL;
    taskEXIT_CRITICAL();
    (void)HAL_UART_AbortTransmit(&huart6);

    taskENTER_CRITICAL();
    start_next();
    taskEXIT_CRITICAL();

    return 0;
}

/* Son giden telemetrinin SEQ'i ve DRDY -> son bit süresi; henüz giden yoksa 0. */
uint32_t raw_link_last(void)
{
    return last_sent;
}

/* USART6 TX bitti: telemetrinin süresini kaydeder, biten satırı bildirir, sıradakini başlatır. */
void raw_link_tx_done_isr(void)
{
    uint32_t now = DWT->CYCCNT;
    BaseType_t woken = pdFALSE;

    if (tx_kind == TX_TELEM)
    {
        uint32_t us = (now - tx_drdy) / (configCPU_CLOCK_HZ / 1000000UL);    // işaretsiz fark CYCCNT taşmasında da doğru

        if (us > 0xFFFFU)
        {
            us = 0xFFFFU;
        }

        last_sent = ((uint32_t)tx_seq << 16) | us;
    }
    else if (tx_kind == TX_LINE && line != NULL && line_pos == line_len)
    {
        line = NULL;
        vTaskNotifyGiveFromISR(line_waiter, &woken);
    }

    start_next();
    portYIELD_FROM_ISR(woken);
}

/* Komut çerçevesinin baytlarını USART1 akışına verir; eksik çerçevede satır ayırıcı uyarılır. */
static void give_command(BaseType_t *woken)
{
    uint8_t seq = rx_frame[2];
    int lost = seq != rx_next;

    if (lost)
    {
        ++rx_lost;
    }

    rx_next = (uint8_t)((seq + 1U) & 0x7FU);
    UartLink_FeedFromISR(&rx_frame[3], rx_frame[1] - 1U, lost, woken);
}

/* Bir baytı mesaja ya da çerçeveye ekler; tamamlananı uygular. Bozuğun son baytı yeninin başı olabilir. */
static void parse_byte(uint8_t byte, BaseType_t *woken)
{
    if (rx_len <= 1U)
    {
        if (rx_len == 1U && (byte == LINUX_MSG_TYPE || byte == FRAME_COMMAND))
        {
            rx_frame[0] = byte;
            rx_len = 2;
            return;
        }

        rx_len = byte == FRAME_SOF ? 1U : 0U;
        return;
    }

    rx_frame[rx_len - 1U] = byte;
    ++rx_len;

    if (rx_frame[0] == LINUX_MSG_TYPE)
    {
        if (rx_len < 4U)
        {
            return;
        }

        if (rx_frame[1] <= 1U && byte == (uint8_t)~rx_frame[1])
        {
            linux_tick = HAL_GetTick();     // linux_on'dan önce: okuyan eski anı görmesin
            linux_on = rx_frame[1];
            rx_len = 0;
            return;
        }
    }
    else if (rx_frame[1] >= 1U && rx_frame[1] <= RX_DATA_MAX)
    {
        if (rx_len < FRAME_OVERHEAD + rx_frame[1])
        {
            return;
        }

        if (byte == Crc8_Compute(rx_frame, 2U + rx_frame[1]))
        {
            give_command(woken);
            rx_len = 0;
            return;
        }
    }

    ++rx_bad;
    rx_len = byte == FRAME_SOF ? 1U : 0U;
}

/* DMA'nın yeni yazdığı baytları işler; yarım, tam ve boşluk olayında çağrılır. */
void raw_link_rx_isr(uint16_t size)
{
    BaseType_t woken = pdFALSE;
    uint32_t end = size % RX_DMA_BYTES;     // tam dolu: konum başa döndü

    while (rx_pos != end)
    {
        parse_byte(rx_dma[rx_pos], &woken);
        rx_pos = (rx_pos + 1U) % RX_DMA_BYTES;
    }

    portYIELD_FROM_ISR(woken);
}

/* Hat hatası: hata almayı, DMA hatası göndermeyi durdurmuşsa yeniden kurar. */
void raw_link_error_isr(void)
{
    ++rx_errors;

    if (huart6.RxState == HAL_UART_STATE_READY)
    {
        rx_len = 0;                     // arada bayt kayboldu
        rx_pos = 0;
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_dma, RX_DMA_BYTES);
    }

    if (tx_kind != TX_IDLE && huart6.gState == HAL_UART_STATE_READY)
    {
        ++tx_drops;                     // çerçeve yarım kaldı; satırı bekleyen task süresini doldurur

        if (tx_kind == TX_LINE)
        {
            line = NULL;
        }

        start_next();
    }
}

/* Linux okuyorsa 1; son mesajdan LINUX_TIMEOUT_MS sonra 0 (meas ve uartTx çağırır). */
int raw_link_linux_on(void)
{
    taskENTER_CRITICAL();               // kesme iki değişkeni de yazar

    if (linux_on && HAL_GetTick() - linux_tick > LINUX_TIMEOUT_MS)
    {
        linux_on = 0;                   // silinir: sayaç taşsa da eski mesaj geçerli olmaz
    }

    int on = (int)linux_on;
    taskEXIT_CRITICAL();

    return on;
}

#endif
