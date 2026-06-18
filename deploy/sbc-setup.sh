#!/usr/bin/env bash
# One-shot system-level setup for the Nano robot SBC (NanoPi NEO Plus2 / Armbian).
# Reproduces every OS change the stack needs so a freshly reflashed board can be
# restored quickly. Run from the repo root:
#
#     sudo bash deploy/sbc-setup.sh
#     sudo reboot                # required: device-tree overlays apply on boot
#
# Then build + run as your normal user (see README §2-4):
#     ~/.pixi/bin/pixi install
#     ~/.pixi/bin/pixi run build
#     ~/.pixi/bin/pixi run bash scripts/stack.sh up
#
# Idempotent — safe to re-run. A backup of armbianEnv.txt is left at *.nano.bak.
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo: sudo bash $0" >&2; exit 1; }

USER_NAME="${SUDO_USER:-ibster}"
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "== 1/4  Device-tree overlays (I2C0/1/2, UART1 for the LDS, ALL USB hosts) =="
ENV=/boot/armbianEnv.txt
# usbhost0..3 = all four H5 host controllers (USB-A + header ports). The OTG/micro-
# USB port (usb@1c19000) is already dr_mode=host in the base DT. Enabling every host
# means a device works from whichever port you plug it into.
NEED="usbhost0 usbhost1 usbhost2 usbhost3 i2c0 i2c1 i2c2 uart1"
cp -n "$ENV" "$ENV.nano.bak" 2>/dev/null || true
if grep -q '^overlays=' "$ENV"; then
  cur=" $(sed -n 's/^overlays=//p' "$ENV") "
  for o in $NEED; do [[ "$cur" == *" $o "* ]] || cur="$cur$o "; done
  sed -i "s|^overlays=.*|overlays=$(echo $cur)|" "$ENV"     # echo trims whitespace
else
  printf 'overlays=%s\n' "$NEED" >> "$ENV"
fi
grep -q '^overlay_prefix=' "$ENV" || echo 'overlay_prefix=sun50i-h5' >> "$ENV"
grep '^overlays=' "$ENV"

echo "== 2/4  udev: non-root I2C access + port-independent USB device names =="
install -m 0644 "$HERE/udev/90-i2c.rules" /etc/udev/rules.d/90-i2c.rules
install -m 0644 "$HERE/udev/95-nano-usb.rules" /etc/udev/rules.d/95-nano-usb.rules
udevadm control --reload-rules || true
udevadm trigger --subsystem-match=i2c-dev || true
udevadm trigger --subsystem-match=tty --subsystem-match=video4linux --subsystem-match=usb || true

echo "== 3/4  groups: $USER_NAME in dialout (serial + i2c) and video (webcam) =="
usermod -aG dialout,video "$USER_NAME"

echo "== 4/5  sudoers: passwordless poweroff/reboot for the web UI Shutdown button =="
install -m 0440 "$HERE/sudoers/nano-power" /etc/sudoers.d/nano-power
[ "$USER_NAME" = ibster ] || sed -i "s/^ibster /$USER_NAME /" /etc/sudoers.d/nano-power
visudo -cf /etc/sudoers.d/nano-power

echo "== 5/5  systemd: start the stack on boot (nano-stack.service) =="
install -m 0644 "$HERE/systemd/nano-stack.service" /etc/systemd/system/nano-stack.service
if [ "$USER_NAME" != ibster ]; then
  sed -i "s|ibster|$USER_NAME|g; s|/home/ibster|$(eval echo "~$USER_NAME")|g" \
    /etc/systemd/system/nano-stack.service
fi
systemctl daemon-reload
systemctl enable nano-stack.service

echo
echo "Done. The stack auto-starts on boot. Build first (pixi install/build), then:"
echo "  sudo reboot   — OR start now with:  sudo systemctl start nano-stack"
