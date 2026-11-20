"""Offline feasibility check: can controller accelerometer dead reckoning bridge constellation gaps?

  python3 -I imu_bridge_check.py <imu-left.csv> <monado window log>

Uses raw constellation samples (WMR_CONSTELLATION_RAW_SAMPLES_LOG) as ground truth where they are
dense, and asks: starting from the constellation position and velocity at t0, how far off is the
predicted position at t0+T for (a) hold, (b) constant velocity, (c) IMU double integration?
Also fits the IMU-to-camera time offset and the yaw offset between the fusion world and the
tracking world, which the IMU method needs.
"""
import bisect
import math
import re
import sys

import numpy as np

imu_path, log_path = sys.argv[1], sys.argv[2]
G = 9.8066

# --- IMU -----------------------------------------------------------------------------------------
imu = np.genfromtxt(imu_path, delimiter=",", names=True)
t_imu = imu["host_ns"].astype(np.float64) / 1e9
acc = np.stack([imu["ax"], imu["ay"], imu["az"]], axis=1)
q = np.stack([imu["qx"], imu["qy"], imu["qz"], imu["qw"]], axis=1)


def rotate(qv, v):
    x, y, z, w = qv[:, 0], qv[:, 1], qv[:, 2], qv[:, 3]
    u = np.stack([x, y, z], axis=1)
    uv = np.cross(u, v)
    uuv = np.cross(u, uv)
    return v + 2.0 * (w[:, None] * uv + uuv)


a_world = rotate(q, acc)
still = np.linalg.norm(acc, axis=1)
print(f"IMU samples {len(t_imu)} over {t_imu[-1] - t_imu[0]:.0f} s, rate {len(t_imu) / (t_imu[-1] - t_imu[0]):.0f} Hz")
rest = np.abs(still - G) < 0.15
print("rotated accel at |a|~g (should be ~(0, 9.8, 0)):", np.round(a_world[rest].mean(axis=0), 3),
      f"(n={rest.sum()})")
lin = a_world - np.array([0.0, G, 0.0])
# Causal bias removal: subtract the mean linear acceleration over the preceding BIAS_S seconds. The
# fusion's ~6 deg tilt error turns gravity into ~1 m/s^2 of fake horizontal acceleration; a hand's
# real acceleration averages out over half a second, the tilt residual does not.
import os
BIAS_S = float(os.environ.get("BIAS_S", "0"))
if BIAS_S > 0:
    cs = np.cumsum(np.vstack([np.zeros(3), lin]), axis=0)
    j0 = np.searchsorted(t_imu, t_imu - BIAS_S)
    idx = np.arange(len(t_imu))
    cnt = np.maximum(idx - j0, 1)
    mean = (cs[idx] - cs[j0]) / cnt[:, None]
    lin = lin - mean

# --- constellation --------------------------------------------------------------------------------
pts = []
for line in open(log_path, errors="replace"):
    m = re.search(r"raw constellation sample .*?t=(\d+) pos=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", line)
    if m:
        pts.append((int(m.group(1)) / 1e9, float(m.group(2)), float(m.group(3)), float(m.group(4))))
pts.sort()
tc = np.array([p[0] for p in pts])
pc = np.array([p[1:] for p in pts])
# Deduplicate identical timestamps (several cameras), keep the first.
keep = np.concatenate([[True], np.diff(tc) > 1e-6])
tc, pc = tc[keep], pc[keep]
print(f"constellation samples {len(tc)} over {tc[-1] - tc[0]:.0f} s; clock delta imu-cam at start "
      f"{(t_imu[0] - tc[0]) * 1000:.0f} ms")


def cam_vel(t, half=0.04):
    """Least-squares slope of constellation position over [t-half, t+half]; None if sparse."""
    i0, i1 = bisect.bisect_left(tc, t - half), bisect.bisect_right(tc, t + half)
    if i1 - i0 < 6:
        return None
    tt = tc[i0:i1] - t
    if tt.min() > -half * 0.6 or tt.max() < half * 0.6:
        return None
    A = np.stack([tt, np.ones_like(tt)], axis=1)
    sol, *_ = np.linalg.lstsq(A, pc[i0:i1], rcond=None)
    return sol[0], sol[1]  # velocity, position at t


def integrate(t0, t1, offset, yaw):
    """Velocity change and position change (with zero initial velocity) from IMU over [t0, t1]."""
    i0 = bisect.bisect_left(t_imu, t0 + offset)
    i1 = bisect.bisect_right(t_imu, t1 + offset)
    if i1 - i0 < 3:
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    a = lin[i0:i1] @ R.T
    tt = t_imu[i0:i1] - offset
    dv = np.zeros(3)
    dp = np.zeros(3)
    prev_t = t0
    for k in range(len(tt)):
        dt = tt[k] - prev_t
        dp += dv * dt + 0.5 * a[k] * dt * dt
        dv += a[k] * dt
        prev_t = tt[k]
    dt = t1 - prev_t
    dp += dv * dt
    return dv, dp


# --- fit time offset and yaw on velocity changes over 150 ms ---------------------------------------
rng = np.random.default_rng(0)
cands = []
for t0 in np.arange(tc[0] + 0.5, tc[-1] - 0.5, 0.05):
    v0 = cam_vel(t0)
    v1 = cam_vel(t0 + 0.15)
    if v0 is not None and v1 is not None and np.linalg.norm(v1[0] - v0[0]) > 0.15:
        cands.append((t0, v1[0] - v0[0]))
print(f"windows with a real velocity change: {len(cands)}")
if len(cands) < 20:
    sys.exit("not enough motion to fit")


def cost(offset, yaw):
    err = 0.0
    n = 0
    for t0, dvc in cands:
        r = integrate(t0, t0 + 0.15, offset, yaw)
        if r is None:
            continue
        err += float(np.sum((r[0] - dvc) ** 2))
        n += 1
    return err / max(n, 1), n


best = None
for off_ms in range(-150, 151, 10):
    for yaw_deg in range(-180, 180, 15):
        c, n = cost(off_ms / 1000, math.radians(yaw_deg))
        if best is None or c < best[0]:
            best = (c, off_ms, yaw_deg)
c0, off0, yaw0 = best
for off_ms in range(off0 - 10, off0 + 11, 2):
    for yaw_deg in range(yaw0 - 15, yaw0 + 16, 2):
        c, n = cost(off_ms / 1000, math.radians(yaw_deg))
        if c < best[0]:
            best = (c, off_ms, yaw_deg)
c_best, off_best, yaw_best = best
c_zero, _ = cost(off_best / 1000, 0.0)
mean_dv = np.mean([np.sum(d ** 2) for _, d in cands])
print(f"fit: imu time offset {off_best} ms, yaw offset {yaw_best} deg; "
      f"velocity-change RMS error {math.sqrt(c_best):.3f} m/s (yaw 0: {math.sqrt(c_zero):.3f}; "
      f"signal RMS {math.sqrt(mean_dv):.3f})")

# --- bridging simulation ----------------------------------------------------------------------------
off, yaw = off_best / 1000, math.radians(yaw_best)
for T in (0.1, 0.2):
    errs = {"hold": [], "const_vel": [], "imu": [], "imu_yaw0": []}
    speeds = []
    for t0 in np.arange(tc[0] + 0.5, tc[-1] - 0.5 - T, 0.02):
        a0 = cam_vel(t0)
        a1 = cam_vel(t0 + T, half=0.03)
        if a0 is None or a1 is None:
            continue
        v0, p0 = a0
        _, p1 = a1
        sp = float(np.linalg.norm(v0))
        r = integrate(t0, t0 + T, off, yaw)
        r0 = integrate(t0, t0 + T, off, 0.0)
        if r is None or r0 is None:
            continue
        speeds.append(sp)
        errs["hold"].append(np.linalg.norm(p0 - p1))
        errs["const_vel"].append(np.linalg.norm(p0 + v0 * T - p1))
        errs["imu"].append(np.linalg.norm(p0 + v0 * T + r[1] - p1))
        errs["imu_yaw0"].append(np.linalg.norm(p0 + v0 * T + r0[1] - p1))
    speeds = np.array(speeds)
    for label, mask in (("all", speeds >= 0), ("speed>0.4 m/s", speeds > 0.4)):
        if mask.sum() < 5:
            continue
        parts = []
        for k in ("hold", "const_vel", "imu", "imu_yaw0"):
            e = np.array(errs[k])[mask] * 100
            parts.append(f"{k} p50 {np.median(e):.1f} / p90 {np.percentile(e, 90):.1f}")
        print(f"T={int(T * 1000)} ms, {label} (n={mask.sum()}): " + " | ".join(parts) + "  [cm]")
