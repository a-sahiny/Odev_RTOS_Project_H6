# MP257_Kopru

STM32F746G-DISCO'daki telemetri istasyonunu (`C:\Users\Asus\Documents\GitHub\Odev_RTOS_Project_H6`) **hiç değiştirmeden** STM32MP257F-DK'ya bağlayan proje.

- **F746**: olduğu gibi. ST-LINK USB'si MP257'nin CN19 USB-A portuna takılır.
- **MP257 Cortex-A35 (Linux)**: `f7linkd` F746'yı `tap-f7` ağ arayüzüne çevirir; `f7hub` F746'nın istemcisidir, M33 ile konuşur, PC'ye web paneli sunar.
- **MP257 Cortex-M33 (RTOS)**: iki ayrı proje — FreeRTOS ve Zephyr. Aynı gözetim çekirdeği (`m33/common`) ve aynı A35↔M33 protokolü.
- **PC**: tarayıcıda `http://192.168.7.1:8080/` ya da `python pc\f7_pc.py`.

**Ana belge:** [docs/01_Proje_Raporu.html](docs/01_Proje_Raporu.html) — mimari, kararlar, adım adım plan (Faz 1–5), FreeRTOS/Zephyr karşılaştırması, eksikler, riskler.

**Animasyonlu anlatım:** [docs/02_Iletisim_Animasyonu.html](docs/02_Iletisim_Animasyonu.html) — mesajların F746, A35, M33 ve PC arasında gerçek baytlarıyla nasıl aktığı (altı senaryo, katman katman paketleme, zamanlama, LED desenleri). Yayımlanmış kopya: https://claude.ai/artifact/R5o32skxFq8uVyHjQEs7s8

## Klasörler

| Klasör | İçerik |
|---|---|
| `linux/` | Kartta çalışan servisler, panel, systemd/udev, kurulum betiği, PC testleri |
| `m33/common/` | Ortak C: satır protokolü + gözetim çekirdeği + PC birim testi |
| `m33/freertos/App/` | FreeRTOS task'ları (CubeIDE projesine bağlanır) |
| `m33/zephyr/f7_supervisor/` | Zephyr uygulaması |
| `tools/` | FreeRTOS projesini oluşturan betik, Zephyr imzalama betiği |
| `pc/` | PC komut satırı istemcisi |
| `protocol/` | A35↔M33 protokol tanımı |
| `docs/` | Rapor ve çalışma notları |

## PC'de testler (kart gerekmez)

```powershell
python -m unittest discover -s C:\Users\Asus\Documents\GitHub\Odev_RTOS_Project_H6\MP257_Kopru\linux\tests -v
```

```powershell
C:\msys64\ucrt64\bin\gcc.exe -std=c11 -Wall -Wextra -Wconversion -I C:\Users\Asus\Documents\GitHub\Odev_RTOS_Project_H6\MP257_Kopru\m33\common C:\Users\Asus\Documents\GitHub\Odev_RTOS_Project_H6\MP257_Kopru\m33\common\link_proto.c C:\Users\Asus\Documents\GitHub\Odev_RTOS_Project_H6\MP257_Kopru\m33\common\supervisor.c C:\Users\Asus\Documents\GitHub\Odev_RTOS_Project_H6\MP257_Kopru\m33\common\tests\test_common.c -o $env:TEMP\test_common.exe; & $env:TEMP\test_common.exe
```
