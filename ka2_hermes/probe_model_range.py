#!/usr/bin/env python3
"""How far the model's predicted route actually reaches.

Read-only: subscribes to the `modelV2` stream that already exists on the box. Nothing is sent.

Reports, over a few seconds:
  * the furthest point in modelV2.position.x, and how many points there are
  * the same for the lane lines, for comparison
  * the car's own speed, so the horizon can be worked out - the model predicts a span of TIME, so a
    route that reaches 100 m at 36 km/h reaches twice that at 72 km/h
"""
import statistics
import sys
import time

sys.path.insert(0, "/data/openpilot")
import cereal.messaging as messaging  # noqa: E402

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0


def reach(flat):
  xs = [float(v) for v in flat]
  return (max(xs) if xs else 0.0), len(xs)


def main():
  sm = messaging.SubMaster(["modelV2", "carState"])
  routes = []
  lanes = []
  speeds = []
  gaps = []
  horizon = 0.0
  npts = 0
  deadline = time.time() + SECONDS
  while time.time() < deadline:
    sm.update(100)
    if not sm.updated["modelV2"]:
      continue
    m = sm["modelV2"]
    rmax, rn = reach(m.position.x)
    if rn:
      routes.append(rmax)
    if len(m.position.x) > 2:
      xs = [float(v) for v in m.position.x]
      gaps.append(statistics.median([b - a for a, b in zip(xs, xs[1:]) if b > a] or [0]))
    try:
      ts = [float(v) for v in m.position.t]
      if len(ts) > 1:
        horizon = max(horizon, ts[-1] - ts[0])
      npts = max(npts, len(m.position.x))
    except Exception:
      pass
    lmax = 0.0
    for line in m.laneLines:
      r, n = reach(line.x)
      lmax = max(lmax, r)
    lanes.append(lmax)
    if sm.updated["carState"]:
      speeds.append(float(sm["carState"].vEgo))

  if not routes:
    print("no model frames in %.0f s - the model only runs while driving" % SECONDS)
    return
  v = statistics.median(speeds) if speeds else 0.0
  rmax = max(routes)
  rmed = statistics.median(routes)
  print("frames                : %d" % len(routes))
  print("speed                 : %.1f m/s (%.0f km/h)" % (v, v * 3.6))
  print("route reach  median   : %.1f m" % rmed)
  print("route reach  furthest : %.1f m" % rmax)
  print("route point spacing   : %.2f m median" % (statistics.median(gaps) if gaps else 0))
  print("route points          : %d, horizon %.1f s" % (npts, horizon))
  print("lane lines   furthest : %.1f m" % (max(lanes) if lanes else 0))
  if v > 1.0:
    print("implied horizon       : %.1f s at the median reach, %.1f s at the furthest"
          % (rmed / v, rmax / v))
    print("at 80 km/h it would reach %.0f m, at 110 km/h %.0f m"
          % (22.2 * (rmed / v), 30.6 * (rmed / v)))


main()
