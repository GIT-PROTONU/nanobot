#!/usr/bin/env bash
# One-shot deploy from the dev host (Windows/git-bash) to the Nano board:
# push src -> rebuild -> restart the runtime stack. Collapses what used to be a
# dozen plink round-trips into a single command.
#
#   NANO_PW=<pw> scripts/deploy.sh                # rebuild everything
#   NANO_PW=<pw> scripts/deploy.sh oled_display   # rebuild only these packages
#
# Creds come from the environment so nothing secret lands in the repo:
#   NANO_PW (required), NANO_HOST, NANO_HOSTKEY, PLINK, PSCP.
set -euo pipefail

HOST=${NANO_HOST:-ibster@192.168.178.141}
PW=${NANO_PW:?set NANO_PW to the board password}
HK=${NANO_HOSTKEY:-'SHA256:F8Ub4q4LFeOegO1MYuY84XfnK05+lx1Rv3TlZHF67Iw'}
PLINK=${PLINK:-'/c/Program Files/PuTTY/plink.exe'}
PSCP=${PSCP:-'/c/Program Files/PuTTY/pscp.exe'}

SEL=""; [ $# -gt 0 ] && SEL="--packages-select $*"

echo ">> pushing src/ + scripts/ to $HOST"
"$PSCP" -batch -pw "$PW" -hostkey "$HK" -r src scripts "$HOST:/home/ibster/Nano/"

echo ">> build + restart on board"
"$PLINK" -batch -pw "$PW" -hostkey "$HK" "$HOST" \
  "cd ~/Nano && ~/.pixi/bin/pixi run bash -c 'colcon build --symlink-install $SEL && bash scripts/stack.sh restart'"
