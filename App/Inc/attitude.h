/* Yönelim süzgeci: jiro + ivme -> temiz ivme / jiro ve roll, pitch, yaw.
   RTOS ve HAL içermez; tek kullanıcısı meas task'ı, PC testi de aynı dosyayı derler. */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define ATT_FLAG_BIAS_READY     0x01U   // jiro bias'ı ölçüldü
#define ATT_FLAG_ACC_USED       0x02U   // bu örnekte ivme, açıyı düzeltti
#define ATT_FLAG_CAL_BUSY       0x08U   // CAL geldi, kartın durması bekleniyor / ölçülüyor
#define ATT_FLAG_CAL_DONE       0x10U   // son CAL başarıyla sıfırladı
#define ATT_FLAG_STILL          0x20U   // kart hareketsiz: yaw kilitli, bias öğreniliyor

/* Bir sensör noktası (FIFO'daki bir paket), sensör ekseninde. */
typedef struct
{
    float acc[3];                       // g
    float gyro[3];                      // rad/s
} AttPoint;

/* Bir çıkış örneği, gövde ekseninde (CAL hizalaması uygulanmış). */
typedef struct
{
    float acc[3];                       // g, alçak geçiren
    float gyro[3];                      // rad/s, bias çıkarılmış, alçak geçiren
    float angle[3];                     // roll, pitch, yaw (rad)
    uint8_t flags;                      // ATT_FLAG_*
} AttOutput;

void att_init(void);                                    // açılış: hizalama yok, bias bilinmiyor
void att_set_rate(float out_hz, float point_hz);        // çıkış ve nokta hızı: kesim frekansları, pencere boyları
void att_restart(void);                                 // START: yaw 0, jiro bias'ı 1 s yeniden ölçülür
void att_calibrate(void);                               // CAL: kart durunca her şey sıfırlanır
/* n nokta -> bir çıkış. dt: iki nokta arası (s); 0 = zaman bilinmiyor, bu blok integre edilmez. */
void att_update(const AttPoint *pt, size_t n, float dt, AttOutput *out);
