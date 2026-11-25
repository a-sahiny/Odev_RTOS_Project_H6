---
tags:
  - paket
aliases:
  - "USART6 çerçevesi"
---

# USART6 çerçevesi

SOF 0xA5, tür, uzunluk, veri (en çok 32 bayt), [[CRC-8]]. F746 → M33: 'T' [[Ham mesaj]], 'D' satır parçası (sıra baytı + en çok 15 karakter). M33 → F746: 'C' komut baytları (sıra baytı + en çok 31).

Ayrı 4 baytlık [[Canlılık mesajı]] aynı hatta gider. Sıra baytı eksik parçayı gösterir; eksik satır atılır, sayılır.

Dosyalar: App/Src/raw_link.c, [[frame.c]], [[f746_tx.c]].
