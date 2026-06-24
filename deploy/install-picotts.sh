#!/usr/bin/env bash
# Install SVOX Pico TTS (pico2wave) for the robot's text-to-speech — ENGLISH +
# GERMAN voices only, to keep the 7 GB rootfs lean. Source build from:
#
#     https://github.com/ihuguet/picotts
#
# Run on the board (or any Debian/Armbian host) from the repo root:
#
#     sudo bash deploy/install-picotts.sh
#
# Builds pico2wave to /usr/local and then PRUNES every lingware except en-US,
# en-GB and de-DE (the es/fr/it data is ~half the install and unused here).
# Playback uses `aplay` (alsa-utils), already present for the webcam mic.
# Idempotent — re-running just rebuilds and re-prunes.
#
# Quick test after install:
#     pico2wave -l de-DE -w /tmp/t.wav "Hallo, ich bin der Nano Roboter" && aplay /tmp/t.wav
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo: sudo bash $0" >&2; exit 1; }

PREFIX="${PREFIX:-/usr/local}"
SRC="${SRC:-/tmp/picotts}"
KEEP_LANGS="en-US en-GB de-DE"        # English + German only

echo "== 1/4  build dependencies (build-only; small) =="
need_pkg=()
command -v git        >/dev/null 2>&1 || need_pkg+=(git)
command -v gcc        >/dev/null 2>&1 || need_pkg+=(build-essential)
command -v autoreconf >/dev/null 2>&1 || need_pkg+=(autoconf automake libtool)
# pico2wave links against popt; aplay comes from alsa-utils.
ls /usr/include/popt.h >/dev/null 2>&1 || need_pkg+=(libpopt-dev)
command -v aplay      >/dev/null 2>&1 || need_pkg+=(alsa-utils)
if [ "${#need_pkg[@]}" -gt 0 ]; then
  echo "  apt-get install: ${need_pkg[*]}"
  apt-get update -qq
  apt-get install -y "${need_pkg[@]}"
fi

echo "== 2/4  fetch + build picotts (ihuguet fork) =="
rm -rf "$SRC"
git clone --depth 1 https://github.com/ihuguet/picotts "$SRC"
cd "$SRC/pico"
if [ -x ./autogen.sh ]; then ./autogen.sh; else autoreconf -fi; fi
./configure --prefix="$PREFIX"
make -j"$(nproc)"
make install
ldconfig || true

echo "== 3/4  prune lingware to English + German only =="
LANGDIR="$PREFIX/share/pico/lang"
shopt -s nullglob
for f in "$LANGDIR"/*.bin; do
  base="$(basename "$f")"; keep=0
  for l in $KEEP_LANGS; do [[ "$base" == "$l"* ]] && keep=1; done
  if [ "$keep" -eq 1 ]; then echo "  keep  $base"; else echo "  prune $base"; rm -f "$f"; fi
done

echo "== 4/4  verify =="
command -v pico2wave
pico2wave -l en-US -w /tmp/nano-tts-test.wav "Nano robot speech is ready" \
  && echo "  synth OK -> /tmp/nano-tts-test.wav  (play with: aplay /tmp/nano-tts-test.wav)"

echo
echo "Done. The web UI 'Speak' button now works once the stack is (re)started."
echo "If you have multiple ALSA outputs, set web_control 'tts_device' in robot.yaml"
echo "(e.g. \"plughw:1,0\" — find it with: aplay -l)."
