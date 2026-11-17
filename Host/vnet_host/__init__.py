"""Host aracı: STM32F746G-DISCO telemetri istasyonunun UART1 üzerinden sanal ağı.

Katmanlar (alttan üste):
    crc        CRC-8/SMBUS (uygulama paketi), CRC-32 (Ethernet FCS)
    proto      uygulama paketi (SOF..CRC8), komutlar, ENV/BULK/CAL, TELEM, cevaplar
    ethframe   Ethernet / ARP / IPv4 / UDP / ICMP üretme ve doğrulama
    lines      UART satırları: FRAME [0|1] <hex>, konsol komutları, JSON
    link       seri port (pyserial) ya da simülatöre bağlı sanal hat
    hoststack  host tarafı ağ yığını: ARP, ping, REQ_ID eşleme, SEQ takibi, raporlar
    accounting GET_STATS sayaçları ve tasarım 18.2 eşitlikleri
    sim_board  kartın protokol düzeyinde Python modeli (kart yokken deneme için)
"""

import os
import sys

__version__ = "1.0"

FROZEN = bool(getattr(sys, "frozen", False))

# Kayıtların (logs/) yazıldığı kök: kaynaktan çalışırken Host klasörü, exe olarak
# çalışırken exe'nin durduğu klasör (PyInstaller'ın geçici açılım klasörü değil).
if FROZEN:
    HOST_DIR = os.path.dirname(os.path.abspath(sys.executable))
else:
    HOST_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
