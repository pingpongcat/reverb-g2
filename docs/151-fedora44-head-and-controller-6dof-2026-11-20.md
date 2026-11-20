# 151 — Fedora 44 on stock NVIDIA: head 6DoF and controller 6DoF in one session (2026-11-20)

One long session on a different machine from the lab rig, bringing the G2 up from nothing on Fedora 44
and then working on tracking quality with the wearer as the verifier. Every visual claim below is the
wearer's report from inside the headset; every number says where it was measured.

**Results, in one place:**

- **Picture without any repo NVIDIA patch.** Fedora's own `monado` package plus stock NVIDIA
  615.78.08 lit the panel through GNOME's Wayland DRM lease, in a 4320x2160 mode at an 11.11 ms
  frame period. `docs/06` says an unpatched driver cannot light 90 Hz; that was measured on 595.
  The refresh rate was not verified independently of the log.
- **Head 6DoF "no different from Windows MR"** with the patched Monado + Basalt from this repo and two
  settings: `SLAM_THREADS=8` and the `docs/130` bounded position horizon at **100 ms**. This is the
  first worn confirmation of the `docs/130` knob. 50 ms visibly froze and then jumped on fast head motion.
- **Controller position tracking from `patches/monado/unapplied/`, for the first time on a headset**,
  plus four new patches (`unapplied/0117-0120`). The wearer's verdict on the final run: "wonderful,
  I can't tell whether Windows looked better; a very small jitter is still visible".
- **The largest single effect was the room, not code:** a PWM LED desk lamp in the cameras' view
  took the tracker from 120 solves/s to 0.2 solves/s.

Limits: one wearer, one controller (the left one), one room, one evening; the A/Bs were sequential
and not blinded, and the hand motion differed between runs.

## 1. Machine and starting point

- Fedora 44 Workstation, GNOME on Wayland. **Fedora 44 ships no X11 session for GNOME**
  (`/usr/share/xsessions/` is empty), so the X11 paths of this repo (`jack-in.sh`, Oasis per
  `docs/138`) are unavailable without installing another desktop.
- RTX 4060 + RTX 2060, NVIDIA 615.78.08 (open kernel module), **no `patches/nvidia/`**.
- `scripts/bootstrap-lab.sh` does not apply: it targets Debian 13 (apt) and refuses to run when a
  driver other than 595.71.05 is loaded. Everything below was done by hand.
- Host Bluetooth adapter: TP-Link UB500 (RTL8761B, `btusb`), as in `docs/136`.

## 2. Bring-up with Fedora packages only

1. `dnf install monado` (25.1.0 snapshot `20261109git4f5025c`). The `openxr` package already ships
   `hello_xr`; `xr-hardware` gives hidraw access through uaccess, so the repo's udev rules were not needed.
2. `~/.config/openxr/1/active_runtime.json` -> `/usr/share/openxr/1/openxr_monado.json`.
3. **`WMR_SLAM=0` is mandatory with this package.** It is built with SLAM on, tries to load
   `libbasalt.so`, fails, and then `wmr_create_headset` fails as a whole:
   `XRT_ERROR_DEVICE_CREATION_FAILED`, so no headset at all.
4. `XRT_NO_STDIN=1 WMR_SLAM=0 WMR_HANDTRACKING=0 monado-service`, then `sleep 180 | hello_xr -g Vulkan2`.

The compositor log: `Available DRM lease device`, `connector DP-3 (HPN)`, then `Lease granted`.
GNOME offered the lease although the connector does **not** carry `non-desktop=1` (that needs
`patches/nvidia/0002`). Consequence: **`preflight.sh` step 3 reports a false NOT READY here.** Step 2
is also a false NOT READY when the controllers are bonded to a host adapter, because it only asks
the headset radio.

## 3. The patched build on Fedora

Built under `~/vr` (the launchers fall back to it via `VR_BASE`). Only `fmt-devel` was missing from
Fedora's default development set.

**Basalt** (pin `df6e970`, `patches/basalt/` 0001-0022, all clean): Pangolin calls
`find_package(Eigen3 NO_MODULE)`, and Fedora 44 ships Eigen **5**. Basalt itself uses its bundled
Eigen 3.x (`BASALT_BUILTIN_EIGEN=ON`). A five-line config file points Pangolin, and later Monado,
at that same bundled copy instead of mixing Eigen 3 and 5:

```cmake
# ~/vr/eigen3-shim/Eigen3Config.cmake
set(EIGEN3_INCLUDE_DIR "$ENV{HOME}/vr/basalt/thirdparty/basalt-headers/thirdparty/eigen")
set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
set(Eigen3_VERSION "3.5.0")
set(Eigen3_FOUND TRUE)
```

`cmake -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBASALT_INSTANTIATIONS_DOUBLE=OFF -DEigen3_DIR=~/vr/eigen3-shim`,
then `ninja basalt` (only the library; the GUI tools were not built).

**Monado** (pin `735e29e4e`, `applies-on-base.txt`): 74 patches apply, but **0083 and 0098 fail even
with `git am -3`** on a fresh clone ("could not build fake ancestor"). 0083's context is
`DEFAULT_CTRL_GAIN 100` from `unapplied/0016`; 0098's is a later rewording of a comment. Both apply
with `patch -p1` at fuzz 1, and the resulting tree was **`3efadad4c…`, byte-identical to the base
tree `unapplied/applies-on-series.txt` names**. Configured with the bootstrap flags plus
`-DEigen3_DIR=~/vr/eigen3-shim`. (`XRT_HAVE_BASALT` is reported unused, as on the lab rig: Basalt is
loaded at run time through `VIT_SYSTEM_LIBRARY_PATH`.)

**Controller build:** a second worktree, `~/vr/monado-ctrl`, with `unapplied/applies-on-series.txt`
on top. All 37 applied with plain `git am`, and the tree was `029544a6…`, as the file states.

## 4. Head 6DoF

Launched with `VR_BASE=~/vr scripts/jack-in-wayland.sh dev 1 6dof`, client
`XR_RUNTIME_JSON=~/vr/monado/build/openxr_monado-dev.json IPC_IGNORE_VERSION=1 hello_xr -g Vulkan2`.

Self-check before the wear test (`~/vr/logs/slam-*/tracking.csv`): 29.4 poses/s, interval p50
33.2 ms, no divergence reset, and the headset resting on the desk stayed within ~2 cm.

| Run | Settings | Wearer | Measured |
|---|---|---|---|
| 1 | launcher defaults (4 threads, no horizon) | "works, but the geometry trembles when I move close to objects" | prediction anchor age p50 74.7 / p90 147.0 ms (`pred: anchor age` lines) |
| 2 | + `docs/130` `horizon50` set (`SLAM_PREDICTION_TYPE=2 SLAM_PRED_FREEZE_POSITION=1 SLAM_PRED_NECK_ARM_MM=0 SLAM_PRED_POSITION_HORIZON_MS=50 SLAM_PRED_POSITION_MAX_SPEED_CM_S=150`) | "much more stable, but on fast motion the update stops and the geometry jumps to the new place" | `timing.csv`: queue before the frontend p90 **53.6 ms**, frontend total p50 30.8 ms per 33 ms frame, camera-to-pose p50 47.5 / p90 **112.4 ms** |
| 3 | `SLAM_THREADS=8`, horizon **100 ms** | "really nice, I see no difference from the Windows Mixed Reality demos" | queue p90 **0 ms**, frontend p50 24.1 ms, camera-to-pose p50 39.2 / p90 **64.4 ms**, 30.1 poses/s |

Run 3 changed two things. The thread count is attributed by `timing.csv` (it removed the queue that
put p90 latency above the 50 ms horizon); the horizon change is attributed by the wearer's run-2
complaint. The launcher's default of 4 threads was tuned on a 6-core machine (T203); on this
24-thread machine 4 left the frontend at ~93% of its frame budget.

## 5. Controllers

### 5.1 Pairing and the right controller

The headset radio reported both controllers `UNPAIRED` (`scripts/controller-pair-check.py`). They had
been bonded to the host UB500 earlier, and pairing them to the headset radio under Windows failed.
After a fresh bond to the host adapter on Fedora, the **left** controller connected and was created
by `wmr_bt_controller.c`. The **right** one never connected (`control_connect_cb ... Host is down`), even
after a power cycle: unresolved. The launcher's guard (`only 1 of 2 bonded controller(s) awake --
launching with those`) handled the dead one correctly in every run.

### 5.2 The applied series registers no controller

With the applied series and `WMR_CONSTELLATION_CONTROLLERS=1` the log said
`Constellation tracker created over 4 tracking cameras (no devices registered yet)`, and only the
Rift and PS Sense drivers call `t_constellation_tracker_add_device`. **The applied series cannot track
controller position at all;** the registration lives in `unapplied/0015a`. The wearer confirmed it:
rotation only.

### 5.3 First run of `unapplied/`: the room

The controller registered (`Registered with constellation tracker (32 LEDs, device id 0)`) and solved
(6 matched blobs, 0.14 px reprojection error), but worn it was mostly lost and jumped when found
(15% of `get_tracked_pose` lines tracked).

Static desk scene (headset on the desk, controller ~45 cm in front, ring towards the cameras;
`WMR_CONSTELLATION_BLOB_TELEMETRY=1 WMR_CONSTELLATION_RAW_SAMPLES_LOG=1`; tools in
`scripts/ctrl-tracking/`), with a PWM LED desk lamp lit above the desk:

| Controller-frame exposure / gain | blobs per frame, 3 of 4 cameras (p50) | raw solves/s |
|---|---|---|
| 6000 µs / 100 (default after 0016) | 69-111 | 0.2 (first capture 0.6) |
| 3000 / 100 | 39-65 | 0.1 |
| 1500 / 100 | 2-4 | 0 |
| 800 / 100 | 1 | 0 |

**No exposure separates the LEDs from the lamp:** exposure and gain scale both alike. Lamp off,
nothing else changed, 6000/100: **8-12 blobs per frame, 120 raw solves/s, 58 of 60 poses tracked
(97%)**. The lab saw 4-15 blobs per frame (`docs/126` §1).

Why Windows does not care about the lamp: it drives a short, bright LED pulse train timed to a short
camera exposure (`docs/126` §3-4), which rejects ambient light optically. Monado sends no pulse train,
so it needs a long exposure, which also collects the lamp. Until that exists on Linux, **the room
must have no bright point lights in the cameras' view.**

One later capture with the same settings got 22 raw solves in two minutes with `monado-service` at
786% CPU; a restart restored 137 solves/s. Lighting was not confirmed either way, so this could be
the room or the "device stuck at zero for the whole session" state of `docs/126` §1.

### 5.4 The jitter and jumps were in the output path → `unapplied/0117`

Worn with the lamp off, the raw solve stream (≈50-120/s) had **no step over 8 cm between consecutive
solves** and a residual of 1.4 mm p50 against a 5-sample mean, but the wearer still saw jitter and jumps.
`wmr_controller_base.c` stores solves in the relation history **without a velocity**, so a query for a
future display time (`m_relation_history_get` -> `m_predict_relation`) returns the newest solve as it
is: the hand lags by the camera latency, freezes at every gap, then steps. And after
`WMR_CONSTELLATION_MAX_AGE_MS` (500 ms) the position becomes invalid, which `hello_xr` shows as the
cube disappearing.

0117 adds a second, output-only history: a one-euro-filtered position, a velocity from consecutive
solves (EMA, reset after a 100 ms gap, speed clamp 3 m/s), and extrapolation capped at
`WMR_CONTROLLER_POS_HORIZON_MS` past the newest solve. The tracker's prior keeps reading the raw history.
Wearer, at 50 ms and one-euro 1.5 Hz / beta 10: **"clearly better, much more stable; still trembles a
little, much less than before; jumps only when I move the hand violently."**

### 5.5 Fast motion: exposure and the prior (`unapplied/0118`)

Raw solve gaps (> 60 ms) bucketed by hand speed over the 50 ms before the gap (`scripts/ctrl-tracking/gaps.py`):

| Run | solves/s | gaps per 100 fast (> 0.8 m/s) solves | median jump on return after a fast gap | rest jitter p50 | Wearer |
|---|---|---|---|---|---|
| B: 0117, 6000/100 | 92 | 3.0 | 29.8 cm | 1.50 mm | "clearly better" |
| C: + 3000 µs / gain 200 | 63 | 1.2 | 16.4 cm | 1.20 mm | "a bit better" |
| D: + `WMR_CONTROLLER_PRIOR_VELOCITY=1` | 75 | 1.8 | 19.0 cm | 1.41 mm | "definitely the best run" |

- 3000/200 halves the motion blur at equal brightness in principle, but gain 200 also raised two
  cameras to 100-118 blobs per frame, and the measured blob brightness dropped (0.80 -> 0.43 mean). It
  works in a dark room; it would not survive the lamp.
- 0118 shifts the tracker's prior along the 0117 velocity (≤ 100 ms): at 1 m/s and 50-80 ms of
  latency the unshifted prior is 5-8 cm behind the hand, against a ±10 cm window (`MIN_POS_ERROR`) and
  an LED projection that has to land on the right blobs. In run D the wearer moved fast three times as
  long (782 fast solves vs ~250), so the numbers are **not** a clean comparison; the wearer's report is
  the only evidence that D beat C.
- CPU was not the limit: no `running slow` or dropped slow samples, busiest thread ~29% (24 threads).

### 5.6 Bridging gaps with the IMU (`unapplied/0119`, `0120`)

0119 logs every fused controller IMU sample (`WMR_CONTROLLER_IMU_CSV=<dir>`). One worn recording
(~90 s, ~200 Hz IMU, ~80 solves/s) analysed offline by `scripts/ctrl-tracking/imu_bridge_check.py`:

- Fusion orientation × accel at rest was (−0.2, 9.59, −1.02) m/s² instead of (0, 9.81, 0): the 3DoF
  fusion's gravity is ~6° off, i.e. ~1 m/s² of fake horizontal acceleration.
- Fitted yaw offset between the fusion world and the tracking world: about −5°, and applying it changed
  nothing. Fitted IMU host-time offset: −8 to −38 ms (weakly determined).
- Predicting 200 ms ahead from the constellation position and velocity, hand above 0.4 m/s (n=26):

| Method | p50 | p90 |
|---|---|---|
| hold | 8.8 cm | 16.1 cm |
| constant velocity | 2.7 cm | 9.0 cm |
| + IMU, no bias removal | 2.2 cm | 7.9 cm |
| + IMU, causal 0.5 s mean removed | **1.7 cm** | **6.1 cm** |

0120 does that live: past the newest solve, report the newest filtered position + velocity × dt + the
double integral of world linear acceleration minus a causal 0.5 s mean, for at most
`WMR_CONTROLLER_IMU_BRIDGE_MS`, then hold; the prior gets the same displacement. Wearer at 250 ms:
**"a very good run, even swinging fast I could not break the geometry off; a slight tremble is still
visible, even on slow motion."**

### 5.7 The remaining tremble: an offline replay picks the filter

`scripts/ctrl-tracking/ctrl_output_replay.py` replays the 0117/0120 output path over the recording at
90 Hz display times (assuming a solve is available 25 ms after its timestamp and the app asks 35 ms
ahead) and measures the shimmer as the residual against the output's own centred 150 ms mean:

| Variant | slow motion (< 0.15 m/s) jitter p50 / p90 | error vs solves, slow / medium speed |
|---|---|---|
| one-euro 1.5/10 + bridge (run above) | 1.06 / 5.38 mm | 3.0 / 11.8 mm |
| 1.5/10, no bridge (horizon 50) | 0.89 / 4.45 mm | 3.0 / 14.2 mm |
| 1.0/5 + bridge | 0.80 / 4.38 mm | 3.2 / 13.4 mm |
| **0.7/3 + bridge** | **0.64 / 3.82 mm** | 3.3 / 15.0 mm |
| 0.5/2 + bridge | 0.53 / 3.37 mm | 3.6 / 16.7 mm |

The bridge adds ~20% shimmer (each new solve resets the integration). Starting the bridge only
60-80 ms after the newest solve changed nothing, because display time is always more than that past
the newest available solve. Wearer at 0.7/3: **"wonderful; I can no longer say whether Windows looked
better. The jitter is very small but still visible."**

What remains, from the wearer: on very fast motion the hand can still outrun the geometry, and rarely,
with the controller held still, the geometry jumps to a slightly different place for one frame.

### 5.8 Two log lines that look like bugs and are not

- `constellation vs IMU ... full=17x deg`: the solve and the raw IMU orientations live in different
  frames; the lab measured 104-161° on 2026-08-12 and the code comment says so. Not a rebase defect.
- `get_tracked_pose: pos=(-1.6, 3.6e22, 0) position_tracked=no`: when the position is invalid,
  `m_predict_relation`'s `do_position` does not write it, so the log prints uninitialised memory. The
  app gets the position-invalid flag and ignores it.

## 6. How to run it

`scripts/start-vr-fedora.sh [up|demo|ctrl|ctrl-demo|down]` carries every setting above (head: 8 threads +
100 ms horizon; `ctrl`: the `~/vr/monado-ctrl` build with 0117-0120 at the best-run values and
3000/200). Environment values override its defaults. `demo` modes run `hello_xr` through
`scripts/ctrl-tracking/demo-loop.sh`, because the controller's Windows/Menu button ends `hello_xr`'s
session and that button sits under the thumb.

Measurement tools, all in `scripts/ctrl-tracking/`: `capture.sh` (restart with telemetry, capture N s,
summarise with `analyze.py`), `gaps.py` (gaps vs hand speed), `imu_bridge_check.py`,
`ctrl_output_replay.py`.

## 7. Next

1. The one-frame jumps with the controller held still: suspected class A of `docs/125` (a correspondence
   slip; 20-26° LED spacing at ~45 cm is the 10-20 cm scale) or the head pose moving the world under the
   controller (the replay does not model head jitter). Log `RAW_SAMPLES_LOG` at rest and look for
   isolated outliers before touching code.
2. The bridge resets at every solve. A small complementary filter (IMU-predicted state corrected
   towards each solve, instead of restarting from it) is the obvious next step for both the shimmer and
   the "outrun on very fast motion".
3. The LED pulse train synchronised to the exposure (`docs/126` §3-4, `docs/143`), the only fix for
   lamps in view.
4. The right controller, and the left one on the headset radio under Windows/Oasis for a native
   comparison on the same hardware.
