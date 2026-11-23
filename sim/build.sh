#!/bin/bash
# mb_via.c compiles from a copy in build/ so the shim headers win over the real ones beside it
# (C searches the including file's dir first). MB_HOST_SIM drops the inter-core glue.
set -e
here="$(cd "$(dirname "$0")" && pwd)"
fw="$here/../firmware"
out="$here/build"
mkdir -p "$out"
cp "$fw/src/mb_via.c" "$here/build/mb_via.c"
cp "$fw/src/mb_speech_bus.c" "$here/build/mb_speech_bus.c"

gcc -Wall -O2 -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" -I"$fw/vendor/emu2149" \
  "$here/host.c" "$here/build/mb_via.c" "$here/build/mb_speech_bus.c" "$fw/src/mb_version.c" "$fw/vendor/emu2149/emu2149.c" \
  -lm -o "$out/host"
echo "built $out/host"

# Without pioasm, sim/mb_bus.pio.h stands in and the PIO interpreter tests are skipped.
gen="$here/build/gen"
rm -rf "$gen"; mkdir -p "$gen"
if command -v pioasm >/dev/null; then
  pioasm -v 1 -o c-sdk "$fw/src/mb_bus.pio" "$gen/mb_bus.pio.h"
  have_pio=1
elif [ "${MB_REQUIRE_PIOASM:-0}" = 1 ]; then
  echo "ERROR: MB_REQUIRE_PIOASM=1 but pioasm is missing" >&2
  exit 1
else
  have_pio=0
fi

# The SDK headers the bus and audio code include, each standing in with sdk_fake.h.
sdk="$here/build/sdk"
rm -rf "$sdk"; mkdir -p "$sdk/hardware" "$sdk/pico"
for h in hardware/clocks hardware/dma hardware/gpio hardware/irq hardware/pio hardware/pwm hardware/sync pico/stdlib; do
  printf '#pragma once\n#include "sdk_fake.h"\n' > "$sdk/$h.h"
done

unit_tests=("$here/test_unit.c")
unit_fw=("$here/build/mb_via.c" "$here/build/mb_speech_bus.c" "$fw/src/mb_version.c")
unit_flags=(-DMB_HOST_SIM -I"$here/shim" -I"$fw/src" -I"$here")

gcc -Wall -O2 "${unit_flags[@]}" "${unit_tests[@]}" "${unit_fw[@]}" -o "$out/test"
echo "built $out/test"

gcc -Wall -O2 -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" -I"$fw/vendor/ssi263" \
  "$here/test_ar.c" "$fw/src/mb_speech.c" "$fw/vendor/ssi263/ssi263.c" "$fw/vendor/ssi263/ssi263dsp.c" \
  -lm -o "$out/test_ar"
echo "built $out/test_ar"

hw_tests=("$here/test_hw.c" "$here/sdk_fake.c" "$here/pio_sim.c")
hw_flags=(-I"$gen" -I"$sdk" -I"$fw/src" -I"$fw/vendor/emu2149" -I"$here")
[ "$have_pio" = 1 ] && hw_flags+=(-DMB_HAVE_PIOASM=1)
gcc -Wall -O2 "${hw_flags[@]}" "${hw_tests[@]}" "$fw/src/mb_bus.c" "$fw/src/mb_audio.c" \
  "$fw/vendor/emu2149/emu2149.c" -lm -o "$out/test_hw"
echo "built $out/test_hw"

echo "---- unit tests (-O2) ----"
"$out/test"
"$out/test_ar"
echo "---- bus + audio + PIO tests (-O2) ----"
"$out/test_hw"
[ "$have_pio" = 1 ] || echo "     (PIO interpreter tests SKIPPED: needs pioasm. The Docker build runs them.)"

# ---- coverage build + gate ----
# Separate -O0 instrumented binaries keep gcov line attribution accurate.
COV_MIN="${COV_MIN:-90}"
covdir="$out/cov"
rm -rf "$covdir"; mkdir -p "$covdir"
cov_lines=0; cov_hit=0
gate_file() {
  local dir="$1" obj="$2" src="$3"
  local cov_out pct total
  cov_out="$(cd "$dir" && gcov -n "$obj" 2>/dev/null)"
  pct="$(printf '%s\n' "$cov_out" \
    | awk -v s="$src" '$0 ~ ("File .*"s){f=1;next} f&&/Lines executed:/{print;exit}' \
    | sed -E "s/.*Lines executed:([0-9.]+)%.*/\1/")"
  if [ -z "$pct" ]; then
    echo "ERROR: could not parse $src coverage from gcov output:" >&2
    printf '%s\n' "$cov_out" >&2
    exit 2
  fi
  printf '%s\n' "$cov_out" | awk -v s="$src" '$0 ~ ("File .*"s){f=1} f{print} f&&/Lines executed:/{exit}'
  total="$(printf '%s\n' "$cov_out" \
    | awk -v s="$src" '$0 ~ ("File .*"s){f=1;next} f&&/Lines executed:/{print;exit}' \
    | sed -E "s/.* of ([0-9]+).*/\1/")"
  cov_lines=$((cov_lines + total))
  cov_hit=$((cov_hit + $(awk -v p="$pct" -v t="$total" 'BEGIN{printf "%d", p * t / 100 + 0.5}')))
  if awk -v p="$pct" -v m="$COV_MIN" 'BEGIN{exit !(p+0 >= m+0)}'; then
    echo "coverage gate PASS: $src ($obj) ${pct}% >= ${COV_MIN}%"
  else
    echo "coverage gate FAIL: $src ($obj) ${pct}% < ${COV_MIN}%" >&2
    exit 1
  fi
}

echo "---- coverage: unit suites ----"
d="$covdir/unit"; mkdir -p "$d"
for f in "${unit_fw[@]}"; do
  gcc -Wall -O0 -g --coverage -DMB_HOST_SIM -I"$here/shim" -I"$fw/src" -c "$f" -o "$d/$(basename "${f%.c}").o"
done
for t in "${unit_tests[@]}"; do
  gcc -Wall -O0 -g "${unit_flags[@]}" -c "$t" -o "$d/unit_$(basename "${t%.c}").o"
done
gcc --coverage "$d"/*.o -o "$out/test_cov"
"$out/test_cov" > /dev/null
gate_file "$d" mb_via.o 'mb_via[.]c'
gate_file "$d" mb_speech_bus.o 'mb_speech_bus[.]c'
gate_file "$d" mb_version.o 'mb_version[.]c'

d="$covdir/speech"; mkdir -p "$d"
gcc -Wall -O0 -g --coverage -DMB_HOST_SIM -I"$here/shim" -I"$fw/src" -I"$fw/vendor/ssi263" \
  -c "$fw/src/mb_speech.c" -o "$d/mb_speech.o"
for f in "$fw/vendor/ssi263/ssi263.c" "$fw/vendor/ssi263/ssi263dsp.c" "$here/test_ar.c"; do
  gcc -O0 -g -DMB_HOST_SIM -I"$here/shim" -I"$fw/src" -I"$fw/vendor/ssi263" -c "$f" -o "$d/$(basename "${f%.c}").o"
done
gcc --coverage "$d"/*.o -lm -o "$out/test_ar_cov"
"$out/test_ar_cov" > /dev/null
gate_file "$d" mb_speech.o 'mb_speech[.]c'

echo "---- coverage: bus + audio ----"
d="$covdir/hw"; mkdir -p "$d"
gcc -Wall -O0 -g --coverage "${hw_flags[@]}" -c "$fw/src/mb_bus.c" -o "$d/mb_bus.o"
gcc -Wall -O0 -g --coverage "${hw_flags[@]}" -c "$fw/src/mb_audio.c" -o "$d/mb_audio.o"
for t in "${hw_tests[@]}" "$fw/vendor/emu2149/emu2149.c"; do
  gcc -Wall -O0 -g "${hw_flags[@]}" -c "$t" -o "$d/hw_$(basename "${t%.c}").o"
done
gcc --coverage "$d"/*.o -lm -o "$out/test_hw_cov"
"$out/test_hw_cov" > /dev/null
gate_file "$d" mb_bus.o 'mb_bus[.]c'
gate_file "$d" mb_audio.o 'mb_audio[.]c'

AINC="-I$here/shim -I$fw/src -I$fw/vendor/emu2149"
echo "---- mixer tests (-O2) ----"
gcc -Wall -O2 -DMB_HOST_SIM $AINC "$here/test_audio.c" "$fw/vendor/emu2149/emu2149.c" -o "$out/test_audio"
"$out/test_audio"

agg="$(awk -v h="$cov_hit" -v t="$cov_lines" 'BEGIN{printf "%.2f", 100 * h / t}')"
if awk -v p="$agg" -v m="$COV_MIN" 'BEGIN{exit !(p+0 >= m+0)}'; then
  echo "coverage gate PASS: aggregate ${agg}% of ${cov_lines} lines >= ${COV_MIN}%"
else
  echo "coverage gate FAIL: aggregate ${agg}% of ${cov_lines} lines < ${COV_MIN}%" >&2
  exit 1
fi

# Optional local test extensions (untracked, absent in the standard tree).
if [ -f "$here/build.local.sh" ]; then source "$here/build.local.sh"; fi
