---
tags:
  - F746
aliases:
  - "FreeRTOS"
---

# FreeRTOS

7 uygulama task'ı + IP task. Öncelik 1–6, tick 1 ms, önleyici zamanlama.

P6 [[critParser]] ve [[ctrlTask]] · P5 [[IP task]] ve [[uartTx]] · P4 [[vnetRx]] · P3 [[meas task]] · P1 lowParser ve worker.
