#!/usr/bin/env bash
# One-shot deploy from the dev host (Windows/git-bash) to the Nano board:
# push src -> rebuild -> restart the runtime stack. Collapses what used to be a
# dozen plink round-trips into a single command.
#
#   NANO_PW=<pw> scripts/deploy.sh                # rebuild everything (+ push devstate/ soul)
#   NANO_PW=<pw> scripts/deploy.sh oled_display   # rebuild only these packages
#   DEPLOY_SOUL=0 NANO_PW=<pw> scripts/deploy.sh  # skip the soul/phrase-bank push
#
# Creds come from the environment so nothing secret lands in the repo:
#   NANO_PW (required), NANO_HOST, NANO_HOSTKEY, PLINK, PSCP.
set -euo pipefail

HOST=${NANO_HOST:-ibster@192.168.178.141}
PW=${NANO_PW:?set NANO_PW to the board password}
HK=${NANO_HOSTKEY:-'SHA256:F8Ub4q4LFeOegO1MYuY84XfnK05+lx1Rv3TlZHF67Iw'}
PLINK=${PLINK:-'/c/Program Files/PuTTY/plink.exe'}
PSCP=${PSCP:-'/c/Program Files/PuTTY/pscp.exe'}
STATE_DIR=/home/ibster/.local/state/nanobot

SEL=""; [ $# -gt 0 ] && SEL="--packages-select $*"

echo ">> pushing src/ + scripts/ to $HOST"
"$PSCP" -batch -pw "$PW" -hostkey "$HK" -r src scripts "$HOST:/home/ibster/Nano/"

# Push the dev-made "soul" (personality.json) + phrase bank (phrases.json) from the project-local
# devstate/ folder onto the board. ON by default for now (DEPLOY_SOUL=1) — so the deployed
# personality is whatever you crafted in devstate/. NOTE: this OVERWRITES the board's own
# personality.json, discarding any trait drift the robot persisted (mood_node saves drift to
# $STATE_DIR/personality.json). Set DEPLOY_SOUL=0 to skip and keep the robot's evolved soul. The
# board regenerates the phrase bank itself when the soul/persona changes, so phrases.json is a
# warm-start only.
if [ "${DEPLOY_SOUL:-1}" = "1" ]; then
  echo ">> pushing devstate/ soul + phrase bank to $HOST:$STATE_DIR (DEPLOY_SOUL=1)"
  "$PLINK" -batch -pw "$PW" -hostkey "$HK" "$HOST" "mkdir -p $STATE_DIR"
  pushed=0
  for f in personality.json phrases.json workshop.json; do
    if [ -f "devstate/$f" ]; then
      "$PSCP" -batch -pw "$PW" -hostkey "$HK" "devstate/$f" "$HOST:$STATE_DIR/$f"
      echo "   copied devstate/$f"
      pushed=1
    fi
  done
  # Skills the dev harness minted in its workshop (devstate/skills/*.md) -> the board's writable
  # learned dir, so deploy carries them alongside the soul/bank + the workshop.json ledger.
  if [ -d "devstate/skills" ] && ls devstate/skills/*.md >/dev/null 2>&1; then
    "$PLINK" -batch -pw "$PW" -hostkey "$HK" "$HOST" "mkdir -p $STATE_DIR/skills"
    "$PSCP" -batch -pw "$PW" -hostkey "$HK" devstate/skills/*.md "$HOST:$STATE_DIR/skills/"
    echo "   copied devstate/skills/*.md"
    pushed=1
  fi
  [ "$pushed" = 0 ] && echo "   (nothing in devstate/ to push — run personality_creator.py first)"
fi

echo ">> build + restart on board"
"$PLINK" -batch -pw "$PW" -hostkey "$HK" "$HOST" \
  "cd ~/Nano && ~/.pixi/bin/pixi run bash -c 'colcon build --symlink-install $SEL && bash scripts/stack.sh restart'"
