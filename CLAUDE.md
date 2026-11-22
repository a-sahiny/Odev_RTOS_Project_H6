# Test çalışma düzeni

Kullanıcının 22.11.2026 tarihli talimatı. Bu depoda her oturumda (Windows, bulut, başka makine) geçerli.

1. **Önce test koşulları.** Bir işe (yeni proje, özellik, hata düzeltmesi) başlarken kod yazmadan önce test koşullarını yaz: neyi doğrulayacağım, hangi girdi/durum, beklenen sonuç, geçti/kaldı ölçütü. Koşulları kullanıcıya göster, sonra koda geç.
2. **Yazdığın kodu kendin test et.** Kullanıcıya test edilmemiş kod verme; derle, çalıştır, testleri koştur. Kartta/donanımda çalışması gereken bir şey buradan sınanamıyorsa bunu açıkça söyle ve tek komutla çalışıp GEÇTİ/KALDI özeti basan bir betik ver.
3. **Testleri raporla.** Her test için: nasıl yazıldı (hangi dosya, hangi yöntem: birim testi, simülatör, gerçek donanım, betik), neyi test etmeyi amaçlıyor, sonucu (geçti/kaldı, sayılar, çıktıdan kısa alıntı). Kalan ve test edilemeyenler de rapora girer, gizlenmez.
4. **Gerçekçi testler, sonsuz test yok.** İmkânsız durumları (donanımın ya da protokolün üretemeyeceği girdiler, gerçekleşemeyecek zamanlamalar) test etme. Test kapsamını baştan sınırla; "bir test daha" döngüsüne girme. Koşullar yazılıp koşulduysa ve geçtiyse dur, raporla.
