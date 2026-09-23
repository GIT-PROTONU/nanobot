#!/usr/bin/env bash
# /proc-based executor-stall trap for app_hub (the 2026-09-20 POST-stall hunt).
#
# Watches every thread of the app_hub process; when any thread sits in
# uninterruptible D-state for >= THRESH seconds it (1) snapshots EVERY thread's
# state + wchan to /tmp/stall_<ts>.txt and (2) signals USR1 to app_hub — the
# in-process faulthandler (app_hub._install_stackdump) then dumps every
# thread's stack to stderr -> journald (`journalctl -u nano-app`), no
# ptrace/root needed. This is the tooling that caught the OLED I2C
# `mv64xxx_i2c_wait_for_completion` D-state on the shared executor (fixed
# 2026-09-22: oled_display's panel I2C moved to a worker thread).
#
# Deployed as a systemd unit (deploy/systemd/nano-stall-trap.service,
# Restart=always): the 30-min self-exit below just re-rolls the trap fresh
# every 30 min (the documented window length), and the unit makes it survive
# reboots — the hand-run /tmp/stall_trap.sh never did (docs/TODO.md). Idle
# cost is ~1 Hz of /proc reads; nothing when app_hub is down.
#
# Env knobs: STALL_THRESH_S (5), STALL_MAX_RUN_S (1800), STALL_POLL_S (1),
# STALL_COOLDOWN_S (60), STALL_OUTDIR (/tmp).
set -u

THRESH_S="${STALL_THRESH_S:-5}"
MAX_RUN_S="${STALL_MAX_RUN_S:-1800}"
POLL_S="${STALL_POLL_S:-1}"
COOLDOWN_S="${STALL_COOLDOWN_S:-60}"
OUTDIR="${STALL_OUTDIR:-/tmp}"

declare -A d_secs=()      # tid -> consecutive D seconds (per pid generation)
last_dump=0
watched_pid=""

log() { echo "[$(date '+%H:%M:%S')] $*"; }

# state char from /proc/<pid>/task/<tid>/stat: comm (2) is parenthesized and may
# contain spaces/parens — strip through the LAST ')' and take the next field.
task_state() { sed 's/.*) //' "$1" 2>/dev/null | cut -d' ' -f1; }

snapshot() {   # $1 = pid, $2 = tid that tripped, $3 = D seconds
  local out="$OUTDIR/stall_$(date +%Y%m%d-%H%M%S).txt"
  {
    echo "app_hub stall trap: pid=$1 tripped tid=$2 D-state ${3}s (threshold ${THRESH_S}s)"
    echo "date: $(date '+%F %T %z')  load: $(cat /proc/loadavg 2>/dev/null)"
    echo "tid  state  wchan  comm"
    for t in /proc/"$1"/task/*; do
      tid=${t##*/}
      st=$(task_state "$t/stat")
      wc=$(cat "$t/wchan" 2>/dev/null)
      cm=$(cat "$t/comm" 2>/dev/null)
      printf '%s %s %s %s\n' "$tid" "${st:-?}" "${wc:-?}" "${cm:-?}"
    done
  } > "$out" 2>/dev/null
  chmod 0644 "$out" 2>/dev/null
  log "D-state >= ${THRESH_S}s (tid $2) — snapshot: $out"
  # The in-process half: every thread's Python stack -> journald.
  kill -USR1 "$1" 2>/dev/null && log "SIGUSR1 sent to app_hub (stacks in journalctl -u nano-app)"
}

end=$((SECONDS + MAX_RUN_S))
log "stall trap up: threshold ${THRESH_S}s, poll ${POLL_S}s, self-exit in ${MAX_RUN_S}s"

while (( SECONDS < end )); do
  pid=$(pgrep -f 'app_hub' 2>/dev/null | head -n1)
  if [ -z "$pid" ]; then
    watched_pid=""; d_secs=()
    sleep "$POLL_S"
    continue
  fi
  if [ "$pid" != "$watched_pid" ]; then
    watched_pid="$pid"; d_secs=()
    log "watching app_hub pid $pid"
  fi

  for t in /proc/"$pid"/task/*; do
    tid=${t##*/}
    st=$(task_state "$t/stat")
    if [ "$st" = "D" ]; then
      d_secs[$tid]=$(( ${d_secs[$tid]:-0} + POLL_S ))
      if (( d_secs[$tid] >= THRESH_S )) && (( SECONDS - last_dump >= COOLDOWN_S )); then
        last_dump=$SECONDS
        snapshot "$pid" "$tid" "${d_secs[$tid]}"
        break                   # one dump per episode is enough
      fi
    else
      [ -n "${d_secs[$tid]+x}" ] && unset "d_secs[$tid]"
    fi
  done
  sleep "$POLL_S"
done

log "stall trap self-exit after ${MAX_RUN_S}s (systemd restarts a fresh window)"
exit 0
