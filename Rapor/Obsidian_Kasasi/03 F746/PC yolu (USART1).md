---
tags:
  - F746
aliases:
  - "PC yolu (USART1)"
---

# PC yolu (USART1)

İlk tasarım: Ethernet frame'leri hex satırı olarak USART1'den (1 Mbaud). [[IP task]], [[uartTx]], [[vnetRx]].

Linux okumuyorken (ve PC hedefinde) kullanılır. Linux okurken satırlar USART6'ya döner ([[Linux tek yol]]); USART1'den bayt çıkmaz. Tek istisna App_Halt'ın son satırı.
