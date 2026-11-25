---
tags:
  - hata
aliases:
  - "Hata: UDP freni"
---

# Hata: UDP freni

Seri port yazımı 1 Mbaud'da kendiliğinden yavaşlar, UDP yavaşlamaz: 21,9 KB 10 ms'de gidince 50 komuttan 18'i ulaştı. Çözüm: Host Linux yolunu 36 KB/s ile sınırlar; sonra 8 patlamada 50/50.

İlgili: [[Host]] · [[telem_udp]] · [[USART6 hattı]].
