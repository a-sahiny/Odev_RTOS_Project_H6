---
tags:
  - hata
aliases:
  - "Hata: arabellek taşması"
---

# Hata: arabellek taşması

Satır parçası küçültülünce 27 baytlık telemetri 20 baytlık arabelleğe yazılacaktı. PC birim testi derlemede yakaladı. Çözüm: arabellek en uzun çerçeveye göre, _Static_assert.

İlgili: [[raw_link]] · [[USART6 çerçevesi]].
