#!/usr/bin/env bash
# ASan + UBSan build of the C ABI and its driver. Run from the repository root.
# FPM cannot inject sanitizer flags per-invocation, so the sources are compiled
# directly here; the C ABI is the contract other languages consume, so that is
# the layer worth sanitizing.
set -euo pipefail

CC_BIN="${CC:-gcc}"
FC_BIN="${FC:-gfortran}"
SAN_FLAGS=(-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1)
OUT_DIR="${TMPDIR:-/tmp}/fabrik-sanitize"
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

echo "== building sanitized objects"
"$FC_BIN" "${SAN_FLAGS[@]}" -J"$OUT_DIR" -c src/fabrik_core.f90 -o "$OUT_DIR/fabrik_core.o"
"$FC_BIN" "${SAN_FLAGS[@]}" -I"$OUT_DIR" -J"$OUT_DIR" -c src/fabrik_c_api.f90 -o "$OUT_DIR/fabrik_c_api.o"
"$CC_BIN" "${SAN_FLAGS[@]}" -Iinclude -c src/fabrik_status.c -o "$OUT_DIR/fabrik_status.o"
"$CC_BIN" "${SAN_FLAGS[@]}" -Iinclude -c ci/sanitize_driver.c -o "$OUT_DIR/sanitize_driver.o"

echo "== linking"
# shellcheck disable=SC2046
"$FC_BIN" "${SAN_FLAGS[@]}" -o "$OUT_DIR/sanitize_driver" \
  "$OUT_DIR/sanitize_driver.o" "$OUT_DIR/fabrik_status.o" \
  "$OUT_DIR/fabrik_c_api.o" "$OUT_DIR/fabrik_core.o" -lm

echo "== running under ASan/UBSan"
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
  "$OUT_DIR/sanitize_driver"
