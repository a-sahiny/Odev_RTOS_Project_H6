---
tags:
  - donanım
aliases:
  - "USART6 hattı"
---

# USART6 hattı

460800 baud, 8N1, iki yönlü. Bir bayt 21,7 µs. Biçim: [[USART6 çerçevesi]].

F746 → M33: [[Ham mesaj]] çerçevesi 27 bayt = 586 µs; satır parçaları en çok 20 bayt = 434 µs, telemetri en çok bir parça bekler. M33 → F746: en çok 39,7 KB/s komut baytı.

M33 tarafında 64 baytlık FIFO açık ([[Hata - UART taşması|Hata: UART taşması]]). F746 almayı dairesel DMA ile yapar.
