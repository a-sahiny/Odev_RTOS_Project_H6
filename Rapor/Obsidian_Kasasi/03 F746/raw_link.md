---
tags:
  - F746
aliases:
  - "raw_link"
---

# raw_link

App/Src/raw_link.c. [[USART6 hattı]]nın F746 ucu. [[Ham mesaj]]'ı ve (Linux okurken) uartTx'in satırlarını [[USART6 çerçevesi]] olarak DMA ile yollar; bekleyen telemetri bir sonraki satır parçasından önce gider. TX bitti kesmesinde DRDY → son bit süresini ölçer ([[Ölçüm - F746|Ölçüm: F746]]).

Alma: dairesel DMA. "Linux var / yok" mesajı ve komut baytları; komut baytları UartLink_FeedFromISR ile USART1'inkilerle aynı akışa girer.
