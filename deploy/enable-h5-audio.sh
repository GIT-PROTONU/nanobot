#!/usr/bin/env bash
# Enable the NanoPi NEO Plus2's INTERNAL analog audio codec (Allwinner H5) for the
# robot's text-to-speech, and make it the default ALSA playback device so the web
# UI's Speak/announce just works. The H5 codec comes up MUTED, so this also un-mutes
# it and persists the mixer state across reboots.
#
#     sudo bash deploy/enable-h5-audio.sh
#     # if it says the codec card isn't present yet: sudo reboot, then re-run.
#
# (The USB webcam mic is a separate capture card and is untouched — `mic_audio.py`
# opens it explicitly, so making the codec the default *playback* device is safe.)
# Idempotent.
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo: sudo bash $0" >&2; exit 1; }

ENV=/boot/armbianEnv.txt

echo "== 1/4  enable the analog-codec device-tree overlay =="
cp -n "$ENV" "$ENV.nano.bak" 2>/dev/null || true
if grep -q '^overlays=' "$ENV"; then
  cur=" $(sed -n 's/^overlays=//p' "$ENV") "
  [[ "$cur" == *" analog-codec "* ]] || sed -i "s|^overlays=.*|& analog-codec|" "$ENV"
else
  echo 'overlays=analog-codec' >> "$ENV"
fi
grep -q '^overlay_prefix=' "$ENV" || echo 'overlay_prefix=sun50i-h5' >> "$ENV"
grep '^overlays=' "$ENV"

# Find the codec's ALSA card (a name containing "codec", e.g. "audiocodec"), skipping
# the USB webcam card. If it's not there yet the overlay needs a reboot to take effect.
CARD="$(sed -n 's/^[[:space:]]*[0-9]\+[[:space:]]*\[\([^ ]*\).*/\1/p' /proc/asound/cards \
        | grep -iv usb | grep -i codec | head -1 || true)"
if [ -z "$CARD" ]; then
  echo
  echo ">> The analog codec card isn't present yet. Reboot to apply the overlay, then"
  echo "   re-run this script to un-mute + set it as default:"
  echo "       sudo reboot   #  ... then:  sudo bash deploy/enable-h5-audio.sh"
  exit 0
fi
echo "   codec card: $CARD"

echo "== 2/4  un-mute every playback control + set a sane level =="
# Control names differ across kernels, so just un-mute/raise all of them (this card
# is playback-only); fine-tune later in `alsamixer -c $CARD` if you like.
while IFS= read -r ctl; do
  [ -n "$ctl" ] || continue
  amixer -c "$CARD" sset "$ctl" unmute   >/dev/null 2>&1 || true
  amixer -c "$CARD" sset "$ctl" 80% >/dev/null 2>&1 || true
done < <(amixer -c "$CARD" scontrols 2>/dev/null | sed "s/^Simple mixer control '\(.*\)',.*/\1/")

echo "== 3/4  make it the default ALSA playback + persist the mixer =="
cat > /etc/asound.conf <<CONF
# Nano robot: default ALSA playback = H5 internal analog codec (card "$CARD").
# Written by deploy/enable-h5-audio.sh — re-run that to regenerate.
pcm.!default {
    type plug
    slave.pcm { type hw; card "$CARD" }
}
ctl.!default { type hw; card "$CARD" }
CONF
alsactl store >/dev/null 2>&1 || true     # saves to /var/lib/alsa; alsa-restore reloads on boot

echo "== 4/4  verify =="
aplay -l | sed 's/^/   /'
echo
echo "Done. Test the speaker:"
echo "   speaker-test -c2 -twav -l1            # a spoken 'front left/right'"
echo "   espeak-ng -w /tmp/t.wav 'Audio is working' && aplay /tmp/t.wav"
echo
echo "web_control 'tts_device' can stay \"\" (it now resolves to this codec via the"
echo "default). To target it explicitly instead, set:  tts_device: \"plughw:CARD=$CARD\""
