#!/usr/bin/env bash
# Build a zenohd router WITH the serial transport (the RoboStack conda libzenohc is
# compiled WITHOUT it, so the stock rmw_zenohd can't speak serial). Uses Docker so no
# Rust toolchain is installed on the host. Must match the conda zenoh version (1.9.0).
#
#   ./build_zenohd_serial.sh x86_64   -> ./zenohd-x86_64  (dev-host testing)
#   ./build_zenohd_serial.sh aarch64  -> ./zenohd-aarch64 (NanoPi H5 board)
set -eux
ARCH="${1:-x86_64}"
OUT="$(cd "$(dirname "$0")/.." && pwd)"   # spike dir
ZTAG=1.9.0

if [ "$ARCH" = "x86_64" ]; then
  docker run --rm -v "$OUT:/out" -w / rust:1-bookworm bash -c "
    apt-get update -qq && apt-get install -y -qq git >/dev/null
    git clone --depth 1 --branch $ZTAG https://github.com/eclipse-zenoh/zenoh /zenoh
    cd /zenoh
    cargo build --release -p zenohd --features zenoh/transport_serial
    cp target/release/zenohd /out/zenohd-x86_64
  "
elif [ "$ARCH" = "aarch64" ]; then
  # Cross-compile for the board. Static musl avoids glibc-version mismatch with Armbian.
  docker run --rm -v "$OUT:/out" -w / rust:1-bookworm bash -c "
    apt-get update -qq && apt-get install -y -qq git gcc-aarch64-linux-gnu >/dev/null
    git clone --depth 1 --branch $ZTAG https://github.com/eclipse-zenoh/zenoh /zenoh
    cd /zenoh
    rustup target add aarch64-unknown-linux-gnu   # add to zenoh's pinned toolchain (after cd)
    CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
      cargo build --release --target aarch64-unknown-linux-gnu -p zenohd --features zenoh/transport_serial
    cp target/aarch64-unknown-linux-gnu/release/zenohd /out/zenohd-aarch64
  "
else
  echo "usage: $0 {x86_64|aarch64}"; exit 2
fi
echo "built: $OUT/zenohd-$ARCH"
