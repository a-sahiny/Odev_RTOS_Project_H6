---
tags:
  - genel
aliases:
  - "Sistem"
---

# Sistem

Bir IMU örneğini F746'dan PC'ye, komutları da PC'den F746'ya taşıyan zincir.

Yukarı: [[IMU]] → [[F746]] → [[Ham mesaj]] → [[USART6 hattı]] → [[M33]] → [[TELEM paketi]] → [[RPMsg]] → [[Linux A35]] → [[telem_udp]] → [[USB ağı]] → [[Host]].

Linux okurken F746'nın cevapları, raporları ve PC'nin komutları da aynı yoldan geçer ([[Linux tek yol]]). Linux yoksa [[PC yolu (USART1)]].

Hız 100 Hz. Kartta 5 dakikada 30.018 paket, kayıp 0 ([[Test - Linux tek yol|Test: Linux tek yol]]).
