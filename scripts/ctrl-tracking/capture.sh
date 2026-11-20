#!/bin/bash
# capture.sh <label> <seconds> [ENV=VAL ...]   (docs/151)
# Restart the controller-tracking Monado build with telemetry + extra env, run hello_xr,
# capture <seconds> of log, then print a summary (analyze.py).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VR_BASE="${VR_BASE:-$HOME/vr}"
LABEL="$1"; SECS="$2"; shift 2
S="${CAPTURE_DIR:-$VR_BASE/logs/captures}"; mkdir -p "$S"
OUT="$S/cap-$LABEL"; mkdir -p "$OUT"
for p in $(pgrep -x hello_xr); do kill "$p"; done
"$REPO/scripts/start-vr-fedora.sh" down >/dev/null 2>&1
sleep 2
env WMR_CONSTELLATION_CONTROLLERS=1 WMR_CONSTELLATION_BLOB_TELEMETRY=1 \
	WMR_CONSTELLATION_RAW_SAMPLES_LOG=1 CONSTELLATION_TRACKER_LOG=info "$@" \
	"$REPO/scripts/start-vr-fedora.sh" ctrl > "$OUT/jackin.out" 2>&1
if ! grep -q 'Socket ready' "$OUT/jackin.out"; then echo "[$LABEL] LAUNCH FAILED"; tail -5 "$OUT/jackin.out"; exit 1; fi
grep -qE 'left: HP Reverb G2 Left' "$VR_BASE/jack-in-wayland.log" || echo "[$LABEL] WARNING: left controller not registered"
(sleep $((SECS + 30)) | XR_RUNTIME_JSON="$VR_BASE/monado-ctrl/build/openxr_monado-dev.json" IPC_IGNORE_VERSION=1 \
	hello_xr -g Vulkan2 > "$OUT/hello_xr.log" 2>&1 &)
sleep 10   # let the session start; not counted
START=$(wc -l < "$VR_BASE/jack-in-wayland.log")
sleep "$SECS"
tail -n +"$START" "$VR_BASE/jack-in-wayland.log" > "$OUT/window.log"
cp "$VR_BASE/jack-in-wayland.log" "$OUT/monado.log"
printf '%s ' "$@" > "$OUT/env.txt"
python3 -I "$REPO/scripts/ctrl-tracking/analyze.py" "$LABEL" "$SECS" "$OUT/window.log"
