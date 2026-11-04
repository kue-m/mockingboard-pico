#!/bin/bash
# mb_via.c compiles from a copy in build/ so the shim headers win over the real ones beside it
# (C searches the including file's dir first). MB_HOST_SIM drops the inter-core glue.
set -e
here="$(cd "$(dirname "$0")" && pwd)"
fw="$here/../firmware"
mkdir -p "$here/build"
cp "$fw/src/mb_via.c" "$here/build/mb_via.c"
cp "$fw/src/mb_speech_bus.c" "$here/build/mb_speech_bus.c"

gcc -Wall -O2 -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" -I"$fw/vendor/emu2149" \
  "$here/host.c" "$here/build/mb_via.c" "$here/build/mb_speech_bus.c" "$fw/src/mb_version.c" "$fw/vendor/emu2149/emu2149.c" \
  -lm -o "$here/host"
echo "built $here/host"

gcc -Wall -O2 -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" \
  "$here/test.c" "$here/build/mb_via.c" "$here/build/mb_speech_bus.c" "$fw/src/mb_version.c" \
  -o "$here/test"
echo "built $here/test"

gcc -Wall -O2 -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" -I"$fw/vendor/ssi263" \
  "$here/test_ar.c" "$fw/src/mb_speech.c" "$fw/vendor/ssi263/ssi263.c" "$fw/vendor/ssi263/ssi263dsp.c" \
  -lm -o "$here/test_ar"
echo "built $here/test_ar"

# -O2 and -O0 code can diverge, so both the correctness build and the coverage build run.
echo "---- unit tests (-O2 correctness build) ----"
"$here/test"
"$here/test_ar"

# ---- coverage build + gate ----
# A separate -O0 instrumented binary so gcov line attribution is accurate.
COV_MIN="${COV_MIN:-80}"
covdir="$here/build/cov"
rm -rf "$covdir"; mkdir -p "$covdir"
gcc -Wall -O0 -g --coverage -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" \
  -c "$here/build/mb_via.c" -o "$covdir/mb_via.o"
gcc -Wall -O0 -g --coverage -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" \
  -c "$here/build/mb_speech_bus.c" -o "$covdir/mb_speech_bus.o"
gcc -Wall -O0 -g --coverage -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" \
  -c "$fw/src/mb_version.c" -o "$covdir/mb_version.o"
gcc -Wall -O0 -g --coverage -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" \
  -c "$here/test.c" -o "$covdir/test.o"
gcc --coverage "$covdir/mb_via.o" "$covdir/mb_speech_bus.o" "$covdir/mb_version.o" "$covdir/test.o" -o "$here/test_cov"
echo "built $here/test_cov"

echo "---- unit tests (coverage build) ----"
"$here/test_cov"

echo "---- coverage report: mb_via.c + mb_speech_bus.c + mb_version.c ----"
gate_file() {
  local dir="$1" obj="$2" src="$3"
  local cov_out pct
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
  if awk -v p="$pct" -v m="$COV_MIN" 'BEGIN{exit !(p+0 >= m+0)}'; then
    echo "coverage gate PASS: $src ${pct}% >= ${COV_MIN}%"
  else
    echo "coverage gate FAIL: $src ${pct}% < ${COV_MIN}%" >&2
    exit 1
  fi
}
gate_file "$covdir" mb_via.o 'mb_via[.]c'
gate_file "$covdir" mb_speech_bus.o 'mb_speech_bus[.]c'
gate_file "$covdir" mb_version.o 'mb_version[.]c'

# mb_speech.c coverage via the A/R regression (engine objects carry no gate).
gcc -Wall -O0 -g --coverage -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" -I"$fw/vendor/ssi263" \
  -c "$fw/src/mb_speech.c" -o "$covdir/mb_speech.o"
gcc -O0 -g -DMB_HOST_SIM -I"$here/shim" -I"$fw/src" -I"$fw/vendor/ssi263" \
  -c "$fw/vendor/ssi263/ssi263.c" -o "$covdir/ssi263.o"
gcc -O0 -g -DMB_HOST_SIM -I"$here/shim" -I"$fw/src" -I"$fw/vendor/ssi263" \
  -c "$fw/vendor/ssi263/ssi263dsp.c" -o "$covdir/ssi263dsp.o"
gcc -Wall -O0 -g --coverage -DMB_HOST_SIM \
  -I"$here/shim" -I"$fw/src" -I"$fw/vendor/ssi263" \
  -c "$here/test_ar.c" -o "$covdir/test_ar.o"
gcc --coverage "$covdir/mb_speech.o" "$covdir/ssi263.o" "$covdir/ssi263dsp.o" "$covdir/test_ar.o" -lm -o "$here/test_ar_cov"
echo "built $here/test_ar_cov"
"$here/test_ar_cov"
gate_file "$covdir" mb_speech.o 'mb_speech[.]c'

# Optional local test extensions (untracked, absent in the standard tree).
if [ -f "$here/build.local.sh" ]; then source "$here/build.local.sh"; fi
