#!/bin/sh
# Kartta çalışır: /home/root/f7kopru'ya kopyalanmış dosyaları yerine koyar ve servisleri açar.
# Kullanım (kartta): sh /home/root/f7kopru/kur.sh
set -e
SRC=/home/root/f7kopru
mkdir -p /usr/local/f7kopru
cp -r "$SRC/f7linkd.py" "$SRC/f7hub.py" "$SRC/f7common" "$SRC/web" /usr/local/f7kopru/
cp "$SRC/udev/99-f746-stlink.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger
cp "$SRC/systemd/f7linkd.service" "$SRC/systemd/f7hub.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now f7linkd.service f7hub.service
systemctl --no-pager status f7linkd.service f7hub.service | head -20
