# A35 ↔ M33 satır protokolü (sürüm 1)

Kanal: RPMsg `rpmsg-tty` uç noktası. Linux'ta `/dev/ttyRPMSG0`, M33'te FreeRTOS varyantında `VIRT_UART0`, Zephyr varyantında `tty_ept`. İki varyant **aynı** protokolü konuşur; Linux tarafındaki `f7hub` hangisinin çalıştığını yalnız `RDY` satırından anlar.

**Neden metin satırı?** Elle denenebilir: kartta `cat /dev/ttyRPMSG0` ile M33'ü dinleyebilir, `echo "HB 1" > /dev/ttyRPMSG0` ile mesaj yollayabilirsin. Hız ihtiyacı küçük (en çok ~120 satır/s).

## Kurallar

- Her satır `\n` ile biter; `\r` yok sayılır. En uzun satır 95 karakter, uzunu sonraki `\n`'e kadar atılır.
- Alanlar **tek boşlukla** ayrılır, hepsi ondalık tam sayı. Alan sayısı ya da aralığı tutmayan satır bütünüyle reddedilir ve `bad` sayacına yazılır.
- Kodu: `m33/common/link_proto.c` (ayrıştırma), `m33/common/supervisor.c` (anlam). PC'de test: `m33/common/tests/test_common.c`.

## A35 → M33

| Satır | Alanlar ve aralık | Ne zaman |
|---|---|---|
| `HB <n>` | n: 0..65535 sayaç | 100 ms'de bir (f7hub `heartbeat`) |
| `TM <seq> <ax> <ay> <az> <gx> <gy> <gz>` | seq 0..65535 (F746 TELEM paketinin SEQ'i), eksenler −32768..32767 ham LSB | her TELEM paketinde |
| `AT <roll> <pitch> <yaw> <flags>` | açılar −36000..36000 (0,01 derece), flags 0..255 | her GET_IMU cevabında (varsayılan 20 ms) |
| `ST <state> <why>` | state 0..255 (bit0 NET_READY, bit1 MEAS_EN, bit2 FAULT), why 0..255 (FAULT nedenleri) | F746 durumu değişince |
| `LIM <cdeg>` | 100..18000 | kanal açılınca, `RDY` gelince, PC'den değişince |

## M33 → A35

| Satır | Anlam |
|---|---|
| `RDY <rtos> <sürüm>` | Açılış: `RDY freertos 1` ya da `RDY zephyr 1` |
| `SUP <mod> <tm_age_ms> <tm_count> <tm_gaps> <dt_min_us> <dt_max_us> <hb_age_ms> <bad>` | 500 ms'de bir gözetim raporu. `dt_*`: son 500 ms'de iki TM arası en kısa/en uzun süre (M33 saatiyle). Hiç gelmediyse yaş 4294967295 |
| `BTN` | USER2 butonuna basıldı (30 ms kararlı). f7hub ölçüm açıksa STOP, kapalıysa START yollar |

## Modlar ve LED3 (turuncu, PH6)

| Kod | Mod | Koşul (öncelik sırasıyla) | LED |
|---|---|---|---|
| 0 | WAIT_A35 | Hiç HB gelmedi | 1 Hz kısa yanıp sönme |
| 5 | A35_LOST | Son HB 500 ms'den eski | 2 s'de bir kısa |
| 4 | F7_FAULT | `ST` satırında FAULT biti | 2,5 Hz |
| 1 | NO_DATA | TM yok ya da 300 ms'den eski | 1 Hz yarı yarıya |
| 3 | ALARM | Son 300 ms içindeki AT'de \|roll\| ya da \|pitch\| > LIM | 10 Hz |
| 2 | RUN | Diğer her durum | Sürekli yanık |

`A35_LOST` M33'ün asıl gerekçesidir: Linux çökse ya da `f7hub` dursa bile M33 bunu 500 ms içinde fark eder ve gösterir.
