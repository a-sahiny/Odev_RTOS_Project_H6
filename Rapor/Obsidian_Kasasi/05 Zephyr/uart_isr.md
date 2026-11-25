---
tags:
  - Zephyr
aliases:
  - "uart_isr"
---

# uart_isr

Baytları [[frame.c]]'ye verir; T1 (çerçevenin ilk baytı) ve T2 (son baytı) damgalarını alır. Telemetri [[k_msgq]]'ya, satır parçası [[line_task]]'a. Yazma kesmesinde [[f746_tx.c]]. Dosya: app_isr.c.
