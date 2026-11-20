#!/bin/bash
# demo-loop.sh [seconds] [runtime-json]: keep hello_xr running for <seconds> total, restarting it
# whenever the session ends (the controller's Windows/Menu button quits hello_xr's session).
END=$(( $(date +%s) + ${1:-900} ))
JSON="${2:-$HOME/vr/monado-ctrl/build/openxr_monado-dev.json}"
while [ "$(date +%s)" -lt "$END" ]; do
	left=$(( END - $(date +%s) ))
	sleep "$left" | XR_RUNTIME_JSON="$JSON" IPC_IGNORE_VERSION=1 hello_xr -g Vulkan2 >> "${DEMO_LOG:-/dev/null}" 2>&1
	echo "$(date +%T) hello_xr exited, restarting" >> "${DEMO_LOG:-/dev/null}"
	sleep 1
done
