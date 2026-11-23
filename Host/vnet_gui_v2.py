"""Grafik arayüz v2 başlatıcısı: python Host/vnet_gui_v2.py  (TelemetriHost_v2.exe'nin giriş noktası)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vnet_host.gui_v2 import main  # noqa: E402

if __name__ == "__main__":
    main()
