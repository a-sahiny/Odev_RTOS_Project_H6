# Devir notu — 22.11.2026 oturumu

Yeni bir oturum bu dosyayı okuyarak kaldığı yerden devam edebilsin diye yazıldı. Depo `a-sahiny/Odev_RTOS_Project_H6`, dal **`claude/dazzling-thompson-c8r0rj`**. Bütün işler bu dalda, `main`'e dokunulmadı.

## Kullanıcının istekleri ve çalışma biçimi
- **Bilmediğini sor.** Kodu okuyarak öğrenilebilecek bir şeyi önce öğren, sonra işe başla.
- **İstenmeyen değişiklik yapma.** Değişiklikler hedefli olsun, istenmeyen özellik eklenmesin.
- **MP257 tarafı adım adım ve birlikte yapılıyor.** Kullanıcı her adımda ne yapıldığını, neden yapıldığını ve nasıl çalıştığını anlamak istiyor.
- **Testleri sen yap.** Kullanıcı "şundan mı bundan mı" diye satır satır komut çalıştırmak istemiyor. Testler önceden, bütün olasılıklarla yapılmalı; kullanıcıya test edilmiş kod verilmeli. Gerekirse tek komutla her şeyi deneyip özet basan bir betik verilmeli.
- **Kartlara erişim.** Kullanıcı Windows PC kullanıyor. F746 kartı şu an PC'ye bağlı, COM6. Bulut oturumu PC'ye ancak Claude Desktop → Remote Control ile klasör paylaşılırsa erişebilir. Bu oturumda paylaşılmış klasör yoktu.

## İstenen değişiklik (tamamlandı, PC'de kartla doğrulandı)
1. **Açıyı host sormuyor.** GET_IMU (0x05) kaldırıldı. Kart her IMU örneğinde (DRDY) bir **TELEM** yollar. TELEM 23 B, big-endian:
   - ax ay az gx gy gz i16 (ham LSB)
   - roll pitch yaw i16 (0,01°)
   - flags u8: bit0 BIAS_READY, bit1 ACC_USED, bit2 PREV
   - prev_seq u16, prev_us u16
2. **Gecikme ölçümü.** Başlangıç DRDY kesmesindeki DWT değeri, bitiş TELEM satırının son bitinin UART'tan çıktığı an (TX bitti kesmesi). Yalnız kart süresi. Bir mesajın süresi bir sonraki TELEM'de gelir (`prev_*`). Hedef 10 ms, sınır 12 ms. Sayaçlar `telem_over10` ve `telem_miss`.
3. **SET_RATE IMU hızını değiştirir:** yalnız 10/20/50/100/200 Hz, varsayılan 100.
   - IMU tarafı: SMPLRT_DIV, gyro ve ivme DLPF'i hıza göre (indeks 6/5/4/3/2).
   - Filtre tarafı: dt = 1/hız, zaman sabiti 0,49 s sabit (alpha = tau/(tau+dt)), bias süresi 1 s.
4. **Hat ve saat.** TELEM satırı ≤ 2 ms olsun diye USART1 **1 Mbaud**: satır 167 karakter = 1,67 ms.
   - F746 **216 MHz**'e alındı (PLL M25 N432 P2 Q9, VOS1 + OverDrive, flash 7 WS, APB1 54 MHz, APB2 108 MHz). Önceden 25 MHz HSE'ydi, kullanıcı "şart değil" dedi.
   - USART1 16x örnekleme. I-cache açık, D-cache kapalı (DMA tamponları önbelleksiz yazılmış).
   - IMU SPI2: ayar /64 = 844 kHz, veri /16 = 3,375 MHz.
5. **İki host:**
   - (a) `Host/` (vnet_gui, CLI, simülatör, T01–T08): MP257 olmadan doğrudan COM ile.
   - (b) `MP257_Kopru/` (f7linkd, f7hub, web paneli, f7_pc.py): köprü üzerinden.
   - İkisi de yeni TELEM'e uyarlandı. f7hub M33'e her TELEM'de `TM ...\nAT ...` satırlarını tek yazımda yolluyor.

## F746 firmware sürümleri ve kartta bulunan sorunlar
| Sürüm | Değişiklik / sebep |
|---|---|
| 0.4 | TELEM her örnekte, SET_RATE = IMU hızı, 1 Mbaud (8x örnekleme, 25 MHz) |
| 0.5 | 216 MHz: 25 MHz'de PC → kart yönünde UART_ORE / line_uart_loss vardı |
| 0.6 | `imu_rate` tanı raporu; START'ta eski DRDY atılıyor (ilk TELEM'de sahte 34–49 ms gecikme vardı) |
| 0.7 | SMPLRT_DIV için bekleme döngüsü. Yanlış teşhisti, 0.8'de kaldırıldı |
| 0.8 | **SPI2 SCK boşta kalıyordu.** SPE kapalıyken PI1'i kimse sürmüyor ve pull-up yok; mod 3'te SPE açılırken CS düşükken fazladan bir saat kenarı oluşuyor, ilk yazım kayıyordu. Artık `prvSetPrescaler` ve SPI abort'tan sonra SPE hemen açılıyor |
| 0.9 | **IMU yeni SMPLRT_DIV'i eski hızdaki bir sonraki örnekte uyguluyor** (10 → 100'de 3 DRDY zaman aşımı, FAULT). Değişimden sonraki ilk DRDY 110 ms bekleniyor |

**0.9 PC'de doğrulandı (COM6):**
- Bütün hızlar ve aralarındaki geçişler çalışıyor.
- DRDY → TX bitti ortalama 1,94 ms, en çok ~4,3 ms; 12 ms aşımı yok.
- imu_*, fault, UART hata sayaçları 0.

`imu_rate` raporu her hız yazımında gidiyor:
`{"type":"imu_rate","hz","w":[div,dlpf,dlpf],"r":[...],"ok"}`.

## MP257 adımları (raporun fazları, `docs/01_Proje_Raporu.html`)
- **1.1 tamam:** Python 3.14.7, gereken bütün stdlib modülleri var. `CONFIG_TUN`, `CONFIG_USB_ACM`, `CONFIG_RPMSG_TTY` modül (`=m`). ip, sysctl, modprobe, udevadm, systemctl tam sürüm.
- **1.2 tamam:** `/dev/ttyACM0`, full-speed USB. ST-LINK kimliği **0483:3752** (374b değil), udev kuralı buna göre güncellendi.
- **2.1 tamam:** dosyalar `/home/root/f7kopru` içinde. Kartın saati UTC'ye ayarlandı, saat dilimi Europe/Istanbul. Saat açılışta kalıcı değil.
- **2.2 YARIM.** `tools/f7_dinle.py /dev/ttyACM0`:
  - F746 → MP257 yönü sağlam: `st` ve `env` satırları geliyor, bozuk satır 0.
  - Betiğin port açılır açılmaz yolladığı **HELLO kayboldu**: F746 `rx_err line_uart_loss` döndü, hello cevabı gelmedi.

## Açık işler (sıradaki oturum)
1. **MP257 → F746 ilk satır kaybı.**
   - Hipotez, doğrulanmadı: `open_serial` önce tcsetattr (eski hız), sonra TCSETS2 (1 Mbaud) yapıyor. cdc_acm her seferinde ST-LINK'e SET_LINE_CODING yolluyor, ST-LINK UART'ını yeniden kurarken hemen yazılan satır bozuluyor.
   - Önerilen sağlam çözüm, sebebe bağlı değil:
     - açtıktan sonra ~100 ms bekle, giriş tamponunu boşalt;
     - ilk satırdan önce bir `\n` yolla (varsa çöp ayrı satır olarak atılsın);
     - `f7linkd` hello raporu gelene kadar HELLO'yu periyodik tekrarlasın.
   - Ölçülecekler: art arda açılışlarda kayıp sürüyor mu? Sürekli bir MP257 → F746 hatası var mı (uzun FRAME satırları, ping)?
2. **EPOCH soğuk açılışta hep 0.** Bu hata bugünkü değişikliklerden önce de vardı.
   - Sebep: `App_Init` önce `SysState_Init`'i çağırıyor, EPOCH orada rastgele sayıdan alınıyor. PRNG'yi tohumlayan ve DWT CYCCNT'yi açan `NetApp_Init` (`prvPrngSeed`) ise daha sonra çalışıyor. Debugger bağlıyken DWT zaten açık olduğu için hata görünmüyor.
   - Etkisi: f7hub F746'nın yeniden başladığını EPOCH değişiminden anlıyor (T3 testi).
   - Kullanıcı açıkça onaylamadı, ama "sorunları sen test edip düzelt" dedi. Düzeltilmeli: EPOCH'u PRNG tohumlandıktan sonra almak (ya da DWT'yi ve tohumu daha erken başlatmak).
3. **Testleri bulutta gerçek parçalarla yap (kullanıcı istedi).** Bu konteynerde TAP çalışıyor: `apt-get install -y iproute2 procps socat` sonrası TAP açıldı ve ayarlandı. Kurulacak düzenek:
   - gerçek `f7linkd` + gerçek TAP + pty üzerinden `Host/vnet_host/sim_board.py` (F746 modeli)
   - gerçek `f7hub` (HTTP / SSE / CSV)
   - `/dev/ttyRPMSG0` yerine pty'ye bağlanan, PC'de derlenmiş `m33/common` çekirdeğiyle sahte M33
   - Bu düzenek ping, START/STOP, 10–200 Hz TELEM, M33 TM/AT/SUP/BTN ve panel akışlarını uçtan uca sınamalı.
   - ST-LINK'e özgü davranışlar (açılıştaki hız değişimi) burada modellenemez. Onlar için kartta tek komutla çalışan, her şeyi deneyip GEÇTİ/KALDI özeti basan bir kabul betiği yazılmalı.
4. **Kullanıcının cevaplamadığı soru:** kart masada düzken pitch ~28,7°. IMU modülü eğik takılı olabilir. ALARM sınırı 30° olduğu için önemli.
5. **CubeMX dosyası.** `.ioc` 216 MHz değerleriyle elle güncellendi. CubeMX'te açılıp Clock Configuration'da HCLK = 216 MHz görünmeli; kod yeniden üretilirse saat ayarı korunmalı.
6. **`Sunum.html`** (kök, ders sunumu) hâlâ eski GET_IMU / 625000 tasarımını anlatıyor, güncellenmedi.
7. **Sonraki fazlar:** 2.2–2.6 (f7linkd ile TAP ve ping, f7hub ve panel, systemd kurulumu), Faz 3 (M33 FreeRTOS), Faz 4 (Zephyr), Faz 5 (kabul testleri).

## Doğrulama komutları (bulut konteyneri)
- **ARM GCC:** `apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi`. Firmware'in tamamı App/Core/Drivers/ThirdParty kaynaklarından `-mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -std=gnu11 -DSTM32F746xx -DUSE_HAL_DRIVER`, `.cproject`'teki include yolları ve `STM32F746NGHX_FLASH.ld` ile derlenip bağlanabiliyor. 0.9: 0 uyarı. `-DAPP_MEAS_SOURCE=1` (IMU'suz) varyantı da derleniyor.
- **Host self-testleri:** `cd Host && python3 -m unittest discover -s selftest` → 22/22.
- **Kabul testleri (simülatörde):** `python3 vnet.py --sim test T01 T02 T03 T04 T05 T06 T08 --seconds 4` hepsi GEÇTİ; T07 `--minutes 1` ile GEÇTİ.
- **Köprü testleri:** `F746_HOST_DIR=$PWD/Host python3 -m unittest discover -s MP257_Kopru/linux/tests` → 5/5.
- **M33 çekirdeği:** `gcc -std=c11 -Wall -Wextra -Wconversion -I MP257_Kopru/m33/common MP257_Kopru/m33/common/{link_proto,supervisor}.c MP257_Kopru/m33/common/tests/test_common.c` → 0 hata.
- **GUI:** `xvfb-run /usr/bin/python3.12` ile açılabiliyor (tkinter yalnız 3.12'de var).
- **Panel:** f7hub HTTP sunucusu sahte TELEM ile çalıştırılıp Playwright ile denenebiliyor.

## Kullanıcı tarafı bilgiler
- **PC depo yolu:** `C:\Users\Asus\Documents\GitHub\Odev_RTOS_Project_H6` (bu dalda). Host komutları `Host` klasöründen çalışıyor (`python vnet.py --port COM6 ...`). `imu` komutu sonda STOP gönderiyor.
- **MP257:** `ssh root@192.168.7.1` (USB ağı, CN15). F746, CN19'a takılınca `/dev/ttyACM0` oluyor.
- **F746'yı yüklemek için** kablo PC'ye takılı olmalı. CubeIDE'de Refresh → Build → Run.
