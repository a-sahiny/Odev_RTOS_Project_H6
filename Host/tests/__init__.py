"""T01–T07 kabul testleri (tasarım 20.2). Her modülde run(host, **ayarlar) -> TestResult."""

from importlib import import_module

TESTS = {
    "T01": ("t01_partial", "Parçalı giriş", "line_rx satır birleştirici, FCS bayrağı"),
    "T02": ("t02_corrupt", "Bozuk giriş", "satır / frame / paket ret nedenleri, yan etkisizlik"),
    "T03": ("t03_slow_consumer", "Yavaş tüketici", "HOLD, sbLow taşması, network buffer pool'u"),
    "T04": ("t04_burst", "Trafik patlaması", "yük altında iki lane, muhasebe, LAT_US"),
    "T05": ("t05_repeat", "Tekrarlanan komut", "REQ_ID dedup, E_CONFLICT"),
    "T06": ("t06_tx_stall", "TX tıkanması", "TX queue'su, kontrol rezervi, TXDELAY"),
    "T07": ("t07_longrun", "Uzun çalışma", "sızıntı, ARP, EPOCH (≥ 30 dk)"),
    "T08": ("t08_imu_deadline", "IMU açıları, GET_IMU deadline", "roll / pitch / yaw, M1 -> M6 ≤ 12 ms"),
}


def get(tid):
    mod, _, _ = TESTS[tid.upper()]
    return import_module("tests." + mod).run
