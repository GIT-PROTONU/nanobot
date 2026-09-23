#!/usr/bin/env bash
# Recurring disk-lean cleanup for the Nano board (NanoPi NEO Plus2, 7 GB rootfs).
#
# The pixi/rattler download cache (~/.cache/rattler/cache) regrows on every
# `pixi install`/resolve and reached 3.7 G (755 package dirs) — 91% of the
# rootfs — while the live env at ~/Nano/.pixi/envs/default keeps its own
# copies. Clearing the cache is regenerable-by-definition and costs nothing
# but a re-download the next time the lockfile actually changes (deploys via
# `pixi run build` use the installed env, no re-resolve). ~/.cache/pip is the
# same story at ~8 M.
#
# Runs as the stack user, ZERO sudo, entirely on user-owned regenerable
# caches — safe to run anytime, even mid-drive (the running stack never
# touches these dirs). Scheduled daily by nano-disk-cleanup.timer
# (deploy/systemd/, installed by deploy/sbc-setup.sh) with Persistent=true,
# so a board that was off at trigger time cleans on the next boot.
#
# NOT touched: ~/Nano/.pixi/envs (the live ROS env), ~/Nano/{build,install},
# brain/, ~/.local/state/nanobot (the persisted soul), /var/*, /usr/*.
#
# Idempotent; a missing cache dir is skipped, not an error. Run manually:
#     bash scripts/disk_cleanup.sh
set -u

usage_df() { df -h / | awk 'NR==2 {print $3" used / "$2", "$5}'; }

before_kb=$(df -k / | awk 'NR==2 {print $3}')
echo "nano disk cleanup start: $(usage_df)"

# The rattler cache: extracted conda packages pixi downloads during
# resolve/install. Hardlink-shared with (or independent of) the live env —
# either way the env holds its own links, so unlinking here is safe.
for d in "$HOME/.cache/rattler/cache/pkgs" \
         "$HOME/.cache/rattler/cache/repodata" \
         "$HOME/.cache/rattler/cache/conda-pypi-mapping" \
         "$HOME/.cache/rattler/cache/uv-cache"; do
  if [ -d "$d" ]; then
    sz=$(du -sh "$d" 2>/dev/null | cut -f1)
    rm -rf "$d" 2>/dev/null || true
    echo "cleared $d ($sz)"
  fi
done

# pip's wheel download cache (small, but free).
if [ -d "$HOME/.cache/pip" ]; then
  sz=$(du -sh "$HOME/.cache/pip" 2>/dev/null | cut -f1)
  rm -rf "$HOME/.cache/pip" 2>/dev/null || true
  echo "cleared $HOME/.cache/pip ($sz)"
fi

after_kb=$(df -k / | awk 'NR==2 {print $3}')
freed_mb=$(( (before_kb - after_kb) / 1024 ))
echo "nano disk cleanup done: freed ~${freed_mb} MB — now $(usage_df)"
