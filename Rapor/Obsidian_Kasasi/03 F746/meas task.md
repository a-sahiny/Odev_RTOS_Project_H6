---
tags:
  - F746
aliases:
  - "meas task"
---

# meas task

Öncelik 3. [[DRDY kesmesi]] ile uyanır, SPI2 DMA ile [[IMU]]'yu okur, [[CAL mailbox]]'a bakar, [[Açı filtresi]]'ni çalıştırır, [[Ham mesaj]]'ı [[raw_link]] ile yollar. START verilmişse ardından [[PC yolu (USART1)]] için paket yollar.
