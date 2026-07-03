#!/usr/bin/env bash
# Install espeak-ng (text-to-speech) for the robot — UK, Lancaster, and Scottish
# voices only, to keep the rootfs lean. Playback uses `aplay` (alsa-utils), already
# present for the webcam mic. Idempotent.
#
# Run on the board from the repo root:
#
#     sudo bash deploy/install-espeakng.sh
#
# Quick test after install:
#     espeak-ng -v en-gb -w /tmp/t.wav "Nano robot speech is ready" && aplay /tmp/t.wav
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo: sudo bash $0" >&2; exit 1; }

echo "== 1/3  install espeak-ng + en-gb voice data =="
apt-get update -qq
apt-get install -y espeak-ng espeak-ng-data

KEEP=0
KEEP_LIST="en-gb en-gb-scotland en-gb-x-gbclan"

echo "== 2/3  prune voices — keep en-gb, Lancaster, Scottish only =="
VOICE_DIR="/usr/share/espeak-ng-data/voices"
if [ -d "$VOICE_DIR" ]; then
  shopt -s nullglob
  for langdir in "$VOICE_DIR"/*/; do
    base="$(basename "$langdir")"
    if [ "$base" != "en" ]; then
      echo "  prune $base"
      rm -rf "$langdir"
    fi
  done
  for variant in "$VOICE_DIR/en/"*; do
    vbase="$(basename "$variant")"
    keep=0
    for k in $KEEP_LIST; do [[ "$vbase" == "$k"* ]] && keep=1; done
    if [ "$keep" -eq 1 ]; then echo "  keep  en/$vbase"; else echo "  prune en/$vbase"; rm -f "$variant"; fi
  done
fi
# Also prune non-kept variant files at the top level
shopt -s nullglob
for f in "$VOICE_DIR"/*; do
  [ -f "$f" ] || continue
  fbase="$(basename "$f")"
  keep=0
  for k in $KEEP_LIST; do [[ "$fbase" == "$k"* ]] && keep=1; done
  if [ "$keep" -eq 1 ]; then echo "  keep  $fbase"; else echo "  prune $fbase"; rm -f "$f"; fi
done

echo "== 3/3  verify =="
command -v espeak-ng
for v in $KEEP_LIST; do
  espeak-ng -v "$v" -w /tmp/nano-tts-test-"$v".wav "Testing the $v voice" \
    && echo "  $v OK -> /tmp/nano-tts-test-$v.wav"
done

echo
echo "Done. The web UI 'Speak' button now works once the stack is (re)started."
echo "If you have multiple ALSA outputs, set web_control 'tts_device' in robot.yaml"
echo "(e.g. \"plughw:1,0\" — find it with: aplay -l)."