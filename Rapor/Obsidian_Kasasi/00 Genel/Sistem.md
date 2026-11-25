---
tags:
  - genel
aliases:
  - "Sistem"
---

# Sistem

Bir IMU örneğini F746'dan PC'ye taşıyan zincir.

Yol: [[IMU]] → [[F746]] → [[Ham mesaj]] → [[USART6 hattı]] → [[M33]] → [[TELEM paketi]] → [[RPMsg]] → [[Linux A35]] → [[telem_udp]] → [[USB ağı]] → [[Host]].

Hız 100 Hz. Kartta uçtan uca 60 saniyede 6003 / 6003 paket, kayıp 0 ([[Test M7]]).

İkinci, daha eski yol: [[PC yolu (USART1)]].
