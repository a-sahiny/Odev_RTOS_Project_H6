"""Grafik arayüz başlatıcısı: python Host/vnet_gui.py  (TelemetriHost.exe'nin giriş noktası)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vnet_host.gui import main  # noqa: E402

if __name__ == "__main__":
    main()
