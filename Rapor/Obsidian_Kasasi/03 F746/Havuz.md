---
tags:
  - F746
aliases:
  - "Havuz"
---

# Havuz

fixed_pool.c: sayan semafor "kaç boş", liste "hangisi boş". Çift bırakma yakalanır. Çalışırken malloc yok.

Önemsiz veri havuzu 8 blok; worker lowParser'dan öncelikli olmalı, yoksa birikmiş kayıtlar havuzu tüketir ([[Hata - T03 havuzu|Hata: T03 havuzu]]).

İlgili: [[FreeRTOS]] · [[Muhasebe]].
