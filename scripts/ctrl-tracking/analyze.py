"""Summarize one constellation capture window: blobs per camera slot, raw samples/s,
tracked share of get_tracked_pose lines, gravity rejects, contested recoveries."""
import collections
import re
import sys

label, secs, path = sys.argv[1], float(sys.argv[2]), sys.argv[3]
frames = collections.defaultdict(list)
raw = tracked = untracked = grav = recovered = 0
for line in open(path, errors="replace"):
    m = re.search(r"push_blobs\] blobs: t=(\d+) n=(\d+)\b", line)
    if m:
        frames[int(m.group(1))].append(int(m.group(2)))
        continue
    if "raw constellation sample" in line:
        raw += 1
    elif "position_tracked=yes" in line:
        tracked += 1
    elif "position_tracked=no" in line:
        untracked += 1
    elif "tracker gravity gate: rejected" in line:
        grav += 1
    elif "RECOVERED via all-blobs" in line:
        recovered += 1

slots = collections.defaultdict(list)
for counts in frames.values():
    for i, n in enumerate(counts):
        slots[i].append(n)
parts = []
for i in sorted(slots):
    s = sorted(slots[i])
    parts.append(f"cam{i} p50 {s[len(s) // 2]:3d}/p90 {s[int(len(s) * 0.9)]:3d}")
total = tracked + untracked
share = f"{100 * tracked / total:.0f}%" if total else "n/a"
print(f"[{label}] raw samples {raw} ({raw / secs:.1f}/s) | tracked {tracked}/{total} ({share}) | "
      f"grav-rejects-logged {grav} | recovered {recovered}")
print(f"[{label}] blobs/frame: " + "  ".join(parts))
