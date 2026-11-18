"""Grafik arayüz v2: gui.App + 'SET_RATE_DEBUG' butonu.

SET_RATE_DEBUG: kaydırıcıdaki hızla SET_RATE paketini bir kez (tekrar denemesiz) gönderir, ardından
host'un hatta yazmasını kilitler: komut tekrarları, ARP cevapları, üreteç, GET_IMU poller'ı ve diğer
butonlar dahil hiçbir bayt gitmez. Alma (RX) sürer, cevaplar ve raporlar görünmeye devam eder.
Kilit 'Debug kilidini kaldır' butonuyla ya da yeni bir bağlantıyla kalkar.
"""

import sys
import threading
import tkinter as tk
from tkinter import ttk

from . import gui
from . import FROZEN, HOST_DIR


class AppV2(gui.App):
    def __init__(self, root):
        self._dbg_locked = False
        self._dbg_owner = None          # kilitliyken tek bir yazıma izinli thread (debug gönderimi)
        self._dbg_tls = threading.local()
        self._dbg_dropped = 0
        super().__init__(root)
        root.title(root.title() + " (v2)")

    def _build(self):
        super()._build()
        btn = self._find_button(self.root, "SET_RATE uygula")
        btn.grid_configure(columnspan=1)
        g, row = btn.master, int(btn.grid_info()["row"])
        ttk.Button(g, text="SET_RATE_DEBUG", command=self._rate_debug).grid(row=row, column=1, sticky="ew",
                                                                            padx=3, pady=3)
        ttk.Button(g, text="Debug kilidini kaldır", command=self._debug_unlock).grid(row=6, column=0, columnspan=2,
                                                                                    sticky="ew", padx=3, pady=3)

    @classmethod
    def _find_button(cls, w, text):
        for c in w.winfo_children():
            if isinstance(c, ttk.Button) and c.cget("text") == text:
                return c
            found = cls._find_button(c, text)
            if found is not None:
                return found
        return None

    # ------------------------------------------------------------ TX kilidi
    def _gate(self, fn):
        def gated(*a, **kw):
            if self._dbg_locked and not getattr(self._dbg_tls, "inside", False):
                if threading.get_ident() != self._dbg_owner:
                    self._dbg_dropped += 1
                    return None
                self._dbg_owner = None                  # tek yazım hakkı kullanıldı
                self._dbg_tls.inside = True             # write_fragmented -> write iç çağrısı geçsin
                try:
                    return fn(*a, **kw)
                finally:
                    self._dbg_tls.inside = False
            return fn(*a, **kw)
        return gated

    def _make_host(self, lk):
        lk.write = self._gate(lk.write)
        lk.write_fragmented = self._gate(lk.write_fragmented)
        return super()._make_host(lk)

    def _attach(self, h):
        self._dbg_locked = False
        self._dbg_owner = None
        super()._attach(h)

    def _bg(self, label, fn, *args):
        if self._dbg_locked:
            self.status_var.set("Debug kilidi açık: host gönderim yapmıyor (%d gönderim engellendi)."
                                % self._dbg_dropped)
            return
        super()._bg(label, fn, *args)

    def _rate_debug(self):
        if self.h is not None and self.h.board_mac is None:
            self.status_var.set("Kart MAC bilinmiyor: önce ARP (aksi halde tek yazım ARP isteğine gider).")
            return
        hz = self.rate_var.get()

        def go():
            self._dbg_dropped = 0
            self._dbg_owner = threading.get_ident()
            self._dbg_locked = True
            r = self.h.set_rate(hz, retries=0)
            self.events.put(("resp_show", ("RATE DEBUG", r)))
            return "SET_RATE %d (debug): %s — host TX kilitli" % (hz, r.status_name)
        super()._bg("SET_RATE %d (debug)…" % hz, go)

    def _debug_unlock(self):
        was = self._dbg_locked
        self._dbg_locked = False
        self._dbg_owner = None
        self.status_var.set("Debug kilidi kaldırıldı (%d gönderim engellenmişti)." % self._dbg_dropped
                            if was else "Debug kilidi zaten kapalı.")


def main():
    if not FROZEN:
        sys.path.insert(0, HOST_DIR)
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    root = tk.Tk()
    AppV2(root)
    root.mainloop()


if __name__ == "__main__":
    main()
