---
tags:
  - Linux
aliases:
  - "telem_udp"
---

# telem_udp

MP257/linux/telem_udp.c. poll ile dört olay: aygıt, 1 s zamanlayıcı, sinyal, komut soketi. TELEM'i UDP 5002'ye, F746'nın satırlarını 5003'e yollar; PC'nin 5003'e yolladığı komut baytlarını M33'e verir. İşlemci payı %0,56 (M7). Çıkış kodları 0, 1, 2.

Host'un Otomatik bağlantısı ssh ile başlatır.
