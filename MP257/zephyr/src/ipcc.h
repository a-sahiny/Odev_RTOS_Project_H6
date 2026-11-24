#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>

extern struct k_sem ipcc_sem;                        // Linux haber verdi: kesmeden link thread'ine
extern volatile uint32_t ipcc_busy;                  // Linux önceki haberi almadığı için atlanan haber
extern volatile uint32_t ipcc_errors;                // sürücünün reddettiği haber

int ipcc_open(void);                                 // IPCC kesmelerini açar; başarı 1
void ipcc_notify(uint32_t channel);                  // Linux'a kesme gönderir, beklemez
void ipcc_isr(const struct device *dev, void *user_data, uint32_t channel, volatile void *data); // IPCC alma kesmesi
