#!/usr/bin/env python3
"""Where did the car actually go? Waypoints from a recent driving segment, for testing road lookups.

Read-only. Prints one coordinate roughly every 20 seconds of driving, so the road-feature query can
be run along roads Kent really uses rather than at town centres.
"""
import glob
import os
import statistics
import sys

sys.path.insert(0, "/data/openpilot")
from openpilot.tools.lib.logreader import LogReader

# newest segments first, but only ones with real driving in them
dirs = sorted(glob.glob("/data/media/0/realdata/2026-*"), reverse=True)[:12]
chosen = None
for d in dirs:
    for p in sorted(glob.glob(os.path.join(d, "rlog.zst")), reverse=True):
        try:
            sp = [float(m.carState.vEgo) for m in LogReader(p) if m.which() == "carState"]
        except Exception:
            continue
        if sp and statistics.median(sp) > 6.0:
            chosen = p
            print("segment: %s  (median %.1f m/s, max %.1f)"
                  % (p.split("/")[-2], statistics.median(sp), max(sp)))
            break
    if chosen:
        break

if not chosen:
    print("no recent driving segment found")
    raise SystemExit(0)

field = None
points = []
n = 0
for m in LogReader(chosen):
    w = m.which()
    if w in ("gpsLocationExternal", "gpsLocation", "liveLocationKalman") and field is None:
        try:
            if w == "liveLocationKalman":
                ll = m.liveLocationKalman.positionGeodetic.value
                lat, lon = float(ll[0]), float(ll[1])
            else:
                g = getattr(m, w)
                lat, lon = float(g.latitude), float(g.longitude)
            field = w
        except Exception as exc:
            print("  %s has no usable position: %s" % (w, exc))
            continue
    if field is None:
        continue
    n += 1
    if n % 200 == 0:                       # roughly every 2 s at 100 Hz
        try:
            if field == "liveLocationKalman":
                ll = m.liveLocationKalman.positionGeodetic.value
                lat, lon = float(ll[0]), float(ll[1])
            else:
                g = getattr(m, field)
                lat, lon = float(g.latitude), float(g.longitude)
            if abs(lat) > 0.0001 and abs(lon) > 0.0001:
                points.append((lat, lon))
        except Exception:
            pass

print("position source: %s, %d usable fixes" % (field, len(points)))
if not points:
    print("NO GPS IN THE LOGS - the box has no position of its own, so the app must supply it")
else:
    step = max(1, len(points) // 12)
    for i in range(0, len(points), step):
        print("   %.5f, %.5f" % points[i])
