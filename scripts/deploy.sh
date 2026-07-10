#!/usr/bin/env bash
# One-shot deploy from the dev host (native Ubuntu) to the Nano board:
# push src -> rebuild -> restart the runtime stack.
#
#   scripts/deploy.sh                # rebuild everything (KEEPS the robot's evolved soul)
#   scripts/deploy.sh oled_display   # rebuild only these packages
#   DEPLOY_SOUL=1 scripts/deploy.sh  # ALSO overwrite the board's soul with memory/
#
# Auth is the passwordless `ssh nano` key alias (see ~/.ssh/config, Host nano) —
# nothing secret needs to land in the repo or the environment. Override the target
# with NANO_HOST (any ssh destination, alias or user@host).
set -euo pipefail

HOST=${NANO_HOST:-nano}
REMOTE_DIR=Nano
STATE_DIR=.local/state/nanobot

SEL=""; [ $# -gt 0 ] && SEL="--packages-select $*"

echo ">> pushing src/ + scripts/ to $HOST:$REMOTE_DIR"
rsync -az --exclude '__pycache__' src scripts "$HOST:$REMOTE_DIR/"

# Push the dev-made "soul" (personality.json) + phrase bank (phrases.json) + hand-editable
# data (presence_chart.yaml, beats.json) from the project-local memory/ folder onto the board.
# OFF by default (DEPLOY_SOUL=0) so the robot KEEPS the personality it has evolved on its own —
# that drift is the whole point of letting it become its own. Opt in with DEPLOY_SOUL=1 to
# OVERWRITE the board's persisted soul with whatever you crafted in memory/ (e.g. a fresh
# personality_creator.py seed, or a hand-edited chart/beat table); this discards the robot's
# accumulated trait drift. The board regenerates the phrase bank itself when the soul/persona
# changes, so phrases.json is only ever a warm-start.
if [ "${DEPLOY_SOUL:-0}" = "1" ]; then
  echo ">> pushing memory/ soul + phrase bank to $HOST:$STATE_DIR (DEPLOY_SOUL=1)"
  ssh "$HOST" "mkdir -p $STATE_DIR"
  pushed=0
  for f in personality.json phrases.json workshop.json trait_history.json self_model.json \
           skill_likes.json presence_chart.yaml beats.json; do
    if [ -f "memory/$f" ]; then
      rsync -az "memory/$f" "$HOST:$STATE_DIR/$f"
      echo "   copied memory/$f"
      pushed=1
    fi
  done
  # Skills the dev harness minted in its workshop (memory/skills/*.md) -> the board's writable
  # learned dir, so deploy carries them alongside the soul/bank + the workshop.json ledger.
  if [ -d "memory/skills" ] && ls memory/skills/*.md >/dev/null 2>&1; then
    ssh "$HOST" "mkdir -p $STATE_DIR/skills"
    rsync -az memory/skills/*.md "$HOST:$STATE_DIR/skills/"
    echo "   copied memory/skills/*.md"
    pushed=1
  fi
  [ "$pushed" = 0 ] && echo "   (nothing in memory/ to push — run personality_creator.py first)"
fi

echo ">> build + restart on board"
ssh "$HOST" \
  "cd $REMOTE_DIR && ~/.pixi/bin/pixi run bash -c 'colcon build --symlink-install $SEL && bash scripts/stack.sh restart'"
