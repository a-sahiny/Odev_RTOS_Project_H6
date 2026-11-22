# Çalışma notları — MP257_Kopru

Bir sonraki oturumda kaynakları yeniden aramadan devam etmek için. Son güncelleme: 22.11.2026.
Kart tarafı genel notları: `C:\Users\Asus\Documents\GitHub\STM32MP257F_DK\docs\00_Calisma_Notlari.md`.

## Durum
- Kod ve plan yazıldı; **kartta hiçbir adım uygulanmadı.** Kullanıcı derleme/flash/kart adımlarını kendisi yapıyor.
- PC'de doğrulanan: `m33/common` gcc testleri (0 hata, `-Wconversion` uyarısız); `linux/tests/test_uyumluluk.py` 5/5 (F746 simülatörüyle uçtan uca); `tools/freertos_projesi_olustur.py --kontrol` 35 bağlantı çözüldü; rapor ve panel 375 px / masaüstü taşmasız.
- Denenmedi: CubeIDE derlemesi, Zephyr derlemesi, TAP ve termios2 (Windows'ta yok), f7hub'un HTTP kısmı (termios import'u Windows'ta çalışmaz).

## F746 tarafından okunan gerçekler (Odev_RTOS_Project_H6)
- USART1 1000000 8N1 (8x oversampling; PCLK2 12,5 MHz'de tam), önceden 625000 → ST-LINK VCP (PA9/PB7). USART6 230400 CubeMX'te açık ama `App/` kullanmıyor.
- IP 192.168.50.2, ağ geçidi 192.168.50.1; UDP 5000 komut, 5001 ENV/BULK/CAL, 5002 telemetri kaynağı. Host portu 6000 (eski host).
- Telemetri ve komut cevapları **gönderenin IP/port'una** gider (`cmd.c:465`, `cmd.c:617`); IP filtresi yok.
- UART satırı: `FRAME 1 <hex+FCS>` (FCS CRC-32 LE), çerçeve 60–256 B (FCS hariç); JSON satırları `{"type":…}`. `st`/`state` raporlarında `ep`, `s` (durum, bit0 NET_READY bit1 MEAS_EN bit2 FAULT), `why`. `fault` raporunda `set`, `why`.
- Konsol: HELLO, STATE, HOLD n, TXDELAY n. GET_IMU (0x05) kaldırıldı (E_TYPE). TELEM 23 B (`>hhhhhhhhhBHH`): ivme, jiro, roll/pitch/yaw, flags, prev_seq, prev_us; her IMU örneğinde bir tane. SET_RATE = IMU hızı: 10/20/50/100/200 Hz (varsayılan 100). prev_us = bir önceki TELEM'in DRDY → TX bitti süresi (kart içi, hedef 10 ms, deadline 12 ms).

## MP257 / CubeMP2
- UM3385: CN19 iki USB-A host (hub üzerinden); LD9 turuncu PH6 aktif yüksek (BSP `LED3`); LD7 kırmızı PH4, LD8 yeşil PH5; USER2 = B4 = PC11 (aktif düşük, BSP `BSP_PB_GetState` ham pin döner).
- CubeMP2'de DK için FreeRTOS+OpenAMP birleşik örnek **yok**: `FreeRTOS_ThreadCreation` (FreeRTOS, TIM6 timebase, IPCC + CoproSync + ShutDown thread, imzalı config) ve `OpenAMP_TTY_echo` (bare-metal) ayrı. libmetal'de `stm32mp-freertos-gcc.cmake` var ama biz `generic` kullanıyoruz (OpenAMP tek task'tan çağrılıyor).
- TTY_echo'ya özgü define'lar: LINUX_RPROC_MASTER, METAL_INTERNAL, METAL_MAX_DEVICE_REGIONS=2, NO_ATOMIC_64_SUPPORT, VIRTIO_DEVICE_SUPPORT=1, VIRTIO_DRIVER_SUPPORT=0, VQ_RX_EMPTY_NOTIFY=0. Ön derleme: `cmake -S …libmetal/ -B libmetal -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=cmake/platforms/stm32mp-generic-gcc.cmake`.
- İmza: `sign_rproc_fw.py --in X_stripped.elf --out X_sign.bin --key Utilities/optee_os/keys/default.pem --plat-tlv BOOTADDR 0x80100000`.

## Zephyr (resmi belge ve kaynak, 21.11.2026'da okundu)
- Kart: `stm32mp257f_dk`, hedef `stm32mp257f_dk/stm32mp257fxx/m33`, "Maintained". A35 desteklenmiyor. Yükleme: Linux remoteproc. **OP-TEE/imza/fw_format hiç geçmiyor.**
- DTS: konsol UART5 (PG9/PG10), USART6 açık; `led0` = LD9 PH6; `sw0` = USER2 PC11 (GPIO_ACTIVE_LOW); `zephyr,flash = &ddr_code` (0x80100000), `zephyr,sram = &ddr_sys` (0x80a00000); mailbox IPCC1 0x40490000.
- `samples/subsys/ipc/openamp_rsc_table`: DK overlay → `zephyr,ipc_shm` 0x81200000 1 MiB, `zephyr,ipc = &mailbox`; prj.conf'ta SHELL_BACKEND_RPMSG açık; uç noktalar `rpmsg-client-sample` ve `rpmsg-tty`; TTY callback tamponu `rpmsg_hold_rx_buffer` ile tutar, thread `rpmsg_send` + `rpmsg_release_rx_buffer`.
- ST topluluğu: bu kartta Zephyr `rpmsg_create_ept` -1 bildirimi (KISMEN).

## Kararlar (ayrıntı raporda K1–K8)
USB VCP bağlantısı · f7linkd seri portun tek sahibi + TAP (MTU 242, IPv6 kapalı, 60 B dolgu) · Linux Python stdlib · f7hub tek istemci + HTTP/SSE · M33 = gözetim · iki RTOS, ortak C çekirdeği · metin satırı protokolü · FreeRTOS projesi betikle.

## Açık işler (kartta teyit)
1. `python3`, `/dev/net/tun`, `cdc_acm` (adım 1.1).
2. ST-LINK PID `374b` ve 1000000 baud'un çalışması (1.2, 2.2; 1 Mbaud PC'de de henüz denenmedi).
3. Birleşik FreeRTOS projesinin ilk derlemesi (3.3).
4. Zephyr Kapı Z1 (imzalı ELF) ve Z2 (RPMsg).
5. LD9/USER2'nin m33-examples imajında M33'e açık olması (ST GPIO_EXTI örneği öyle kullanıyor).
