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
#     ~/.pixi/bin/pixi run build && ~/.pixi/bin/pixi run build-lds
#     ~/.pixi/bin/pixi run bash scripts/stack.sh up
#
# Idempotent — safe to re-run. A backup of armbianEnv.txt is left at *.nano.bak.
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo: sudo bash $0" >&2; exit 1; }

USER_NAME="${SUDO_USER:-ibster}"
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "== 1/4  Device-tree overlays (I2C0/1/2, UART1 for the LDS, USB host) =="
ENV=/boot/armbianEnv.txt
NEED="usbhost1 usbhost2 i2c0 i2c1 i2c2 uart1"
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

echo "== 2/4  udev: non-root I2C access via the dialout group =="
install -m 0644 "$HERE/udev/90-i2c.rules" /etc/udev/rules.d/90-i2c.rules
udevadm control --reload-rules || true
udevadm trigger --subsystem-match=i2c-dev || true

echo "== 3/4  groups: $USER_NAME in dialout (serial + i2c) and video (webcam) =="
usermod -aG dialout,video "$USER_NAME"

echo "== 4/4  sudoers: passwordless poweroff/reboot for the web UI Shutdown button =="
install -m 0440 "$HERE/sudoers/nano-power" /etc/sudoers.d/nano-power
[ "$USER_NAME" = ibster ] || sed -i "s/^ibster /$USER_NAME /" /etc/sudoers.d/nano-power
visudo -cf /etc/sudoers.d/nano-power

echo
echo "Done. Now: sudo reboot   (then build + run — see README)."
