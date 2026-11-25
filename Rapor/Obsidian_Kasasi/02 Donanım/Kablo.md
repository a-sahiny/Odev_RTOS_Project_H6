---
tags:
  - donanım
aliases:
  - "Kablo"
---

# Kablo

F746 D1 (PC6, TX) → MP257 CN5 pin 10 (PF14, RX): telemetri ve satırlar.

MP257 CN5 pin 8 (PF13, TX) → F746 D0 (PC7, RX): "Linux var / yok" mesajı ve PC'nin komutları (25.11.2026'da takıldı). GND → CN5 pin 6.

CN5 pin 2 ve 4 5 V'tur: önce iki kart da kapalıyken takılır. Açarken önce MP257, sonra F746.

İki hat da 3,3 V (belgeden; multimetre yoktu).

İlgili: [[USART6 hattı]] · [[F746]] · [[MP257]].
