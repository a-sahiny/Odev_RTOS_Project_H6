"""Başlatıcı: python Host/vnet.py <komut>  (her klasörden çalışır). Ayrıntı: python Host/vnet.py -h"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vnet_host.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
