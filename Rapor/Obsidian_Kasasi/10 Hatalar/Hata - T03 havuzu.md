---
tags:
  - hata
aliases:
  - "Hata: T03 havuzu"
---

# Hata: T03 havuzu

T03'te 12 kayıttan 4'ü pool_low_empty ile düşüyordu. lowParser ve worker aynı öncelikteydi (1): HOLD'un vTaskDelay'i tick sınırında bitince lowParser 1 ms'lik diliminde 8 bloklu havuzu tüketiyordu. Çözüm: worker önceliği 2. Önceki imajlarda da vardı.

İlgili: [[FreeRTOS]] · [[Havuz]] · [[T01–T08]].
