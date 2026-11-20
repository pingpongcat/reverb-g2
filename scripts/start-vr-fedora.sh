#!/bin/bash
# start-vr-fedora.sh -- the launcher of docs/151: patched Monado (head 6DoF, optionally controller
# position) with the settings confirmed in the headset on 2026-11-20 on a Fedora 44 machine
# (24 threads, stock NVIDIA 615). Expects the builds of docs/151 under $VR_BASE (default ~/vr):
# monado/ (applies-on-base.txt), monado-ctrl/ (+ unapplied/applies-on-series.txt), basalt/.
#
#   start-vr-fedora.sh             stable build: head 6DoF, controllers rotation only
#   start-vr-fedora.sh demo        the same, plus hello_xr for 10 minutes
#   start-vr-fedora.sh ctrl        experimental build (~/vr/monado-ctrl): controller position too
#   start-vr-fedora.sh ctrl-demo   the same, plus hello_xr for 10 minutes
#   start-vr-fedora.sh down        stop everything
#
# Wake the controllers first (the launcher waits up to 30 s for them).
# Controller position needs a room without bright point lights in the cameras' view
# (a PWM LED desk lamp gave 70-110 blobs per frame and 0.2 solves/s; off: 8-12 and 120/s).
#
# Why these values (measured from ~/vr/logs/slam-*/timing.csv, controller-less desk + worn):
#   SLAM_THREADS=8: with the repo default of 4, the Basalt frontend took ~31 ms per 33 ms
#     camera frame, frames queued (p90 53.6 ms) and camera-to-pose latency hit p90 112 ms.
#     With 8: no queue, p90 64 ms.
#   Bounded position horizon (docs/130): stops geometry jitter when approaching objects.
#     50 ms froze and then jumped on fast head motion; 100 ms felt like Windows MR.
# Controller settings (ctrl), worn A/B with the user on 2026-11-20, best run so far ("may match
# Windows MR; jitter very small but visible; could not break tracking with fast swings"):
#   output path (unapplied/0117): bounded prediction + one-euro, 0.7 Hz / beta 3 (offline
#     replay: slow-motion jitter p50 1.06 -> 0.64 mm vs 1.5/10, a few mm more lag mid-speed);
#   prior velocity (0118); IMU gap bridge 250 ms (0120);
#   exposure 6000 us / gain 100: 3000/200 helped in a dark room (less blur) but with a lamp in
#   view it raised blobs to 125-250 per frame (3.6 solves/s vs 245/s at 6000/100, 2026-11-21).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export VR_BASE="${VR_BASE:-$HOME/vr}"
MODE="${1:-up}"
BUILD="$VR_BASE/monado"

case "$MODE" in
down)
	for p in $(pgrep -x hello_xr); do kill "$p"; done
	exec "$REPO/scripts/jack-in-wayland.sh" down
	;;
up|demo|ctrl|ctrl-demo)
	CTRL_ENV=()
	if [ "$MODE" = ctrl ] || [ "$MODE" = ctrl-demo ]; then
		BUILD="$VR_BASE/monado-ctrl"
		CTRL_ENV=(
			JACK_IN_SERVICE="$BUILD/build/src/xrt/targets/service/monado-service"
			WMR_CONSTELLATION_CONTROLLERS=1
			WMR_CONTROLLER_POS_HORIZON_MS="${WMR_CONTROLLER_POS_HORIZON_MS:-50}"
			WMR_CONTROLLER_POS_FILTER_MIN_CUTOFF="${WMR_CONTROLLER_POS_FILTER_MIN_CUTOFF:-0.7}"
			WMR_CONTROLLER_POS_FILTER_BETA="${WMR_CONTROLLER_POS_FILTER_BETA:-3}"
			WMR_CONTROLLER_IMU_BRIDGE_MS="${WMR_CONTROLLER_IMU_BRIDGE_MS:-250}"
			WMR_CONTROLLER_PRIOR_VELOCITY="${WMR_CONTROLLER_PRIOR_VELOCITY:-1}"
			WMR_CONTROLLER_CAM_EXPOSURE_US="${WMR_CONTROLLER_CAM_EXPOSURE_US:-6000}"
			WMR_CONTROLLER_CAM_GAIN="${WMR_CONTROLLER_CAM_GAIN:-100}"
		)
	fi
	rm -f "${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/monado_comp_ipc"
	env SLAM_THREADS=8 \
		SLAM_PREDICTION_TYPE=2 SLAM_PRED_FREEZE_POSITION=1 SLAM_PRED_NECK_ARM_MM=0 \
		SLAM_PRED_POSITION_HORIZON_MS=100 SLAM_PRED_POSITION_MAX_SPEED_CM_S=150 \
		"${CTRL_ENV[@]}" \
		"$REPO/scripts/jack-in-wayland.sh" up 1 6dof || exit
	echo "Clients: XR_RUNTIME_JSON=$BUILD/build/openxr_monado-dev.json IPC_IGNORE_VERSION=1 <app>"
	if [ "$MODE" = demo ] || [ "$MODE" = ctrl-demo ]; then
		# Loop: the controller's Windows/Menu button ends hello_xr's session; restart it.
		"$REPO/scripts/ctrl-tracking/demo-loop.sh" 600 "$BUILD/build/openxr_monado-dev.json"
	fi
	;;
*)
	echo "usage: $0 [up|demo|ctrl|ctrl-demo|down]" >&2
	exit 2
	;;
esac
