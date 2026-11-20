"""Offline replay of the controller output path (lab-ctrl commits fc2759291, 72cafe967).

  python3 -I ctrl_output_replay.py <imu-left.csv> <monado window log>

Replays raw constellation solves + IMU through the same steps as wmr_controller_base.c
(one-euro on the solved position, velocity EMA, IMU bridge) and samples the output at 90 Hz display
times, as an app would. Reports, for slow hand motion, the frame-to-frame "jitter": the residual
of the output against its own centred 150 ms moving average (what the eye sees as shimmer), plus
the lag of the output behind the raw solves. Assumptions, both rough: a solve becomes available
AVAIL_MS after its timestamp, and the app asks for LEAD_MS past "now".
"""
import bisect
import math
import os
import re
import sys

import numpy as np

imu_path, log_path = sys.argv[1], sys.argv[2]
AVAIL_MS = float(os.environ.get("AVAIL_MS", "25"))
LEAD_MS = float(os.environ.get("LEAD_MS", "35"))
G = 9.8066

imu = np.genfromtxt(imu_path, delimiter=",", names=True)
t_imu = imu["host_ns"].astype(np.int64)
acc = np.stack([imu["ax"], imu["ay"], imu["az"]], axis=1)
q = np.stack([imu["qx"], imu["qy"], imu["qz"], imu["qw"]], axis=1)
u = q[:, :3]
w = q[:, 3:4]
uv = np.cross(u, acc)
a_world = acc + 2.0 * (w * uv + np.cross(u, uv))
a_world[:, 1] -= G

pts = []
for line in open(log_path, errors="replace"):
    m = re.search(r"raw constellation sample .*?t=(\d+) pos=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", line)
    if m:
        pts.append((int(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))))
pts.sort()
dedup = []
for p in pts:
    if not dedup or p[0] > dedup[-1][0]:
        dedup.append(p)
tc = np.array([p[0] for p in dedup], dtype=np.int64)
pc = np.array([p[1:] for p in dedup])


class OneEuro:
    def __init__(self, fc_min, fc_d, beta):
        self.fc_min, self.fc_d, self.beta = fc_min, fc_d, beta
        self.prev = None

    @staticmethod
    def alpha(fc, dt):
        tau = 1.0 / (2 * math.pi * fc)
        return 1.0 / (1.0 + tau / dt)

    def run(self, ts, y):
        if self.prev is None:
            self.prev, self.prev_dy, self.prev_ts = y.copy(), np.zeros(3), ts
            return y.copy()
        dt = (ts - self.prev_ts) / 1e9
        self.prev_ts = ts
        if dt <= 0:
            return self.prev.copy()
        dy = (y - self.prev) / dt
        ad = self.alpha(self.fc_d, dt)
        self.prev_dy = ad * dy + (1 - ad) * self.prev_dy
        fc = self.fc_min + self.beta * np.linalg.norm(self.prev_dy)
        a = self.alpha(fc, dt)
        self.prev = a * y + (1 - a) * self.prev
        return self.prev.copy()


def replay(fc_min, beta, bridge_ms, horizon_ms=50, tau_ms=40, bias_s=0.5, off_ms=-15, max_speed=3.0, start_ms=0):
    # IMU linear accel with causal EMA bias, as wmr_controller_base_push_bridge_imu.
    lin = np.zeros_like(a_world)
    bias = a_world[0].copy()
    for k in range(len(t_imu)):
        if k > 0 and bias_s > 0:
            dt = (t_imu[k] - t_imu[k - 1]) / 1e9
            bias += (a_world[k] - bias) * min(dt / bias_s, 1.0)
        lin[k] = a_world[k] - bias if bias_s > 0 else a_world[k]
    imu_cam_ts = t_imu - int(off_ms * 1e6)

    # Output history: per solve, filtered position and velocity.
    filt = OneEuro(fc_min, 1.0, beta) if fc_min > 0 else None
    out_ts, out_pos, out_vel, out_vv = [], [], [], []
    last_ts, last_pos, vel, vv = None, None, np.zeros(3), False
    for ts, p in zip(tc, pc):
        gap = None if last_ts is None else ts - last_ts
        if filt is not None:
            if gap is not None and gap > 250e6:
                filt = OneEuro(fc_min, 1.0, beta)
            pf = filt.run(int(ts), p)
        else:
            pf = p.copy()
        if gap is not None and gap <= 100e6:
            dt = gap / 1e9
            vi = (pf - last_pos) / dt
            if np.linalg.norm(vi) > 2 * max_speed:
                if not vv:
                    vel = np.zeros(3)
            elif vv and tau_ms > 0:
                vel = vel + (vi - vel) * (1 - math.exp(-dt / (tau_ms / 1000)))
            else:
                vel = vi
            sp = np.linalg.norm(vel)
            if sp > max_speed:
                vel = vel * max_speed / sp
            vv = True
        else:
            vel, vv = np.zeros(3), False
        last_ts, last_pos = ts, pf
        out_ts.append(ts)
        out_pos.append(pf)
        out_vel.append(vel.copy())
        out_vv.append(vv)
    out_ts = np.array(out_ts)

    # Frames at 90 Hz.
    frames = np.arange(tc[0] + int(1e9), tc[-1] - int(0.2e9), int(1e9 / 90))
    res_t, res_p = [], []
    for now in frames:
        avail = now - int(AVAIL_MS * 1e6)
        i = bisect.bisect_right(out_ts, avail) - 1
        if i < 0:
            continue
        newest = out_ts[i]
        if now - newest > 500e6:
            continue
        query = now + int(LEAD_MS * 1e6)
        p0, v0 = out_pos[i], out_vel[i] if out_vv[i] else np.zeros(3)
        if bridge_ms > 0 and (query - newest) <= start_ms * 1e6:
            pos = p0 + v0 * (query - newest) / 1e9
        elif bridge_ms > 0:
            to = min(query, newest + int(bridge_ms * 1e6))
            j0 = bisect.bisect_right(imu_cam_ts, newest + int(start_ms * 1e6))
            j1 = bisect.bisect_right(imu_cam_ts, min(to, now))  # only IMU already received
            t = newest + int(start_ms * 1e6)
            dp, v = v0 * (t - newest) / 1e9, v0.copy()
            for j in range(j0, j1):
                dt = (imu_cam_ts[j] - t) / 1e9
                dp += v * dt + 0.5 * lin[j] * dt * dt
                v += lin[j] * dt
                t = imu_cam_ts[j]
            dp += v * max(to - t, 0) / 1e9
            pos = p0 + dp
        else:
            dt = min(query - newest, int(horizon_ms * 1e6)) / 1e9
            pos = p0 + v0 * dt if out_vv[i] else p0
        res_t.append(query)
        res_p.append(pos)
    res_t = np.array(res_t)
    res_p = np.array(res_p)

    # Raw-solve truth at the query times (interpolated), only where solves are dense around it.
    jit, lag, spd = [], [], []
    half = int(0.075e9)
    for k in range(len(res_t)):
        t = res_t[k]
        a, b = bisect.bisect_left(res_t, t - half), bisect.bisect_right(res_t, t + half)
        if b - a < 10:
            continue
        centred = res_p[a:b].mean(axis=0)
        i0, i1 = bisect.bisect_left(tc, t - half), bisect.bisect_right(tc, t + half)
        if i1 - i0 < 8:
            continue
        span = (tc[i1 - 1] - tc[i0]) / 1e9
        if span < 0.1:
            continue
        s = np.linalg.norm(pc[i1 - 1] - pc[i0]) / span
        truth = np.array([np.interp(t, tc[i0:i1], pc[i0:i1, d]) for d in range(3)])
        jit.append(np.linalg.norm(res_p[k] - centred) * 1000)
        lag.append(np.linalg.norm(res_p[k] - truth) * 1000)
        spd.append(s)
    jit, lag, spd = np.array(jit), np.array(lag), np.array(spd)
    out = {}
    for name, m in (("slow<0.15", spd < 0.15), ("mid0.15-0.5", (spd >= 0.15) & (spd < 0.5))):
        if m.sum() > 20:
            out[name] = (np.median(jit[m]), np.percentile(jit[m], 90), np.median(lag[m]), m.sum())
    return out


raw_jit = []
for i in range(3, len(tc) - 3):
    if (tc[i + 3] - tc[i - 3]) / 1e6 > 80:
        continue
    raw_jit.append(np.linalg.norm(pc[i] - pc[i - 3:i + 4].mean(axis=0)) * 1000)
print(f"solves {len(tc)}, IMU {len(t_imu)}; raw solve residual vs 7-sample mean p50 {np.median(raw_jit):.2f} mm")
variants = [
    ("current: euro 1.5/10, bridge 250", 1.5, 10, 250, 0),
    ("euro 1.5/10, bridge 250 after 60 ms", 1.5, 10, 250, 60),
    ("euro 1.5/10, bridge 250 after 80 ms", 1.5, 10, 250, 80),
    ("euro 0.7/3, bridge 250", 0.7, 3, 250, 0),
    ("euro 0.7/3, bridge 250 after 60 ms", 0.7, 3, 250, 60),
    ("euro 1.0/5, bridge 250 after 60 ms", 1.0, 5, 250, 60),
]
for label, fc, beta, br, st in variants:
    r = replay(fc, beta, br, start_ms=st)
    parts = [f"{k}: jitter p50 {v[0]:.2f}/p90 {v[1]:.2f} mm, err-vs-solve p50 {v[2]:.1f} mm (n={v[3]})"
             for k, v in r.items()]
    print(f"{label:40s} " + " | ".join(parts))
