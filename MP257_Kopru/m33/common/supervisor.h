/* M33 gözetim çekirdeği: A35 kalp atışı, telemetri tazeliği ve aralığı, eğim alarmı, buton, LED deseni.
   RTOS'suz saf C. Zaman girişi mikrosaniye (uint32, taşma farkla doğru hesaplanır); tek task çağırır. */
#pragma once
#include <stdint.h>
#include <stddef.h>

#include "link_proto.h"

#define SUP_HB_TIMEOUT_US     500000U       // 500 ms: A35 hub 100 ms'de bir HB yollar, 4 kayba izin
#define SUP_TM_TIMEOUT_US     300000U       // 300 ms: en düşük hız 10 Hz'de 3 paket kaybı
#define SUP_AT_TIMEOUT_US     300000U       // açı bu kadar eskiyse alarm kararına girmez
#define SUP_LIM_DEFAULT_CDEG  3000          // 30,00 derece
#define SUP_REPORT_US         500000U       // SUP satırı periyodu
#define SUP_BTN_STABLE        3U            // 10 ms örnekle 30 ms kararlı basış = bir basış

typedef enum
{
    SUP_WAIT_A35 = 0,                       // henüz hiç HB gelmedi
    SUP_NO_DATA,                            // HB var, telemetri yok ya da eski
    SUP_RUN,                                // telemetri akıyor, alarm yok
    SUP_ALARM,                              // |roll| ya da |pitch| sınırı aştı
    SUP_F7_FAULT,                           // F746 FAULT bitini bildirdi
    SUP_A35_LOST                            // HB vardı, kesildi (Linux çöktü ya da hub durdu)
} SupMode;

typedef struct
{
    SupMode mode;
    int32_t lim_cdeg;

    uint8_t hb_seen;
    uint32_t hb_us;                         // son HB zamanı

    uint8_t tm_seen;
    uint32_t tm_us;                         // son TM zamanı
    uint16_t tm_seq;                        // son TM SEQ'i (boşluk sayımı için)
    uint32_t tm_count;                      // açılıştan beri TM
    uint32_t tm_gaps;                       // SEQ boşluklarından kayıp TM
    uint32_t dt_min_us;                     // rapor penceresinde en kısa TM aralığı
    uint32_t dt_max_us;                     // rapor penceresinde en uzun TM aralığı

    uint8_t at_seen;
    uint32_t at_us;
    int32_t roll_cdeg;
    int32_t pitch_cdeg;

    uint8_t f7_state;                       // ST satırındaki durum baytı (bit2 = FAULT)
    uint8_t f7_why;

    uint32_t bad_lines;                     // ayrıştırılamayan satır
    uint32_t report_us;                     // son SUP satırı zamanı

    uint8_t btn_count;                      // ardışık basılı örnek
    uint8_t btn_latched;                    // bu basış bildirildi
} Supervisor;

void sup_init(Supervisor *s, uint32_t now_us);                         // başlangıç durumu
void sup_on_msg(Supervisor *s, const LinkMsg *msg, uint32_t now_us);    // A35'ten gelen mesaj
void sup_on_bad_line(Supervisor *s);                                   // bozuk satır sayacı
SupMode sup_step(Supervisor *s, uint32_t now_us);                      // modu günceller ve döndürür
int sup_button_sample(Supervisor *s, int pressed);                     // yeni basışta 1
int sup_led_on(SupMode mode, uint32_t now_us);                         // LED bu anda yanmalı mı
int sup_report_due(Supervisor *s, uint32_t now_us);                    // SUP satırı zamanı geldiyse 1
size_t sup_format_report(Supervisor *s, uint32_t now_us, char *out, size_t cap); // SUP satırı, '\n' dahil; pencereyi sıfırlar
