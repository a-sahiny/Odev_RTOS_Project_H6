/* IPCC posta kutusu: Linux'a haber gönderir; gelen haber ipcc_isr ile semafora dönüşür. */
#include "ipcc.h"
#include <zephyr/drivers/ipm.h>
#include <soc.h>
#include <stm32_ll_ipcc.h>

volatile uint32_t ipcc_busy;
volatile uint32_t ipcc_errors;

static const struct device *const ipm = DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));
static IPCC_TypeDef *const regs = (IPCC_TypeDef *)DT_REG_ADDR(DT_CHOSEN(zephyr_ipc));

/* IPCC kesmelerini açar; başarı 1, sürücü hazır değilse 0. */
int ipcc_open(void)
{
    if (!device_is_ready(ipm))
    {
        return 0;
    }

    ipm_register_callback(ipm, ipcc_isr, NULL);

    return ipm_set_enabled(ipm, 1) == 0;
}

/* Linux'a bir kanaldan kesme gönderir. */
void ipcc_notify(uint32_t channel)
{
    // ipm_send dolu kanalda boş döner; bekleyen haber Linux'u zaten uyandıracak
    if (LL_C2_IPCC_IsActiveFlag_CHx(regs, LL_IPCC_CHANNEL_1 << channel))
    {
        ++ipcc_busy;
        return;
    }

    if (ipm_send(ipm, 0, channel, NULL, 0) != 0)
    {
        ++ipcc_errors;
    }
}
