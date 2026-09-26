#!/usr/bin/env python3
"""Where does lane centring actually lose accuracy?

Reads one recorded drive (qlog/rlog) and measures the things that decide centring quality:

  * the lateral offset of the lane centre from the car, at a few distances ahead
  * how much that offset oscillates, and at what frequency - hunting shows up here
  * how late the steering is relative to the offset - a short lookahead shows up here
  * how often the lane pair is missing or faint, which caps everything else
  * the speed, so distances can be turned into the time the car is really looking ahead

Read-only. Usage: centering_study.py <segment path or route dir>
"""
import math
import statistics
import sys

sys.path.insert(0, "/data/openpilot")
try:
  from openpilot.tools.lib.logreader import LogReader
except Exception:
  from tools.lib.logreader import LogReader

PATH = sys.argv[1]
LOOK_M = (5.0, 10.0, 20.0, 30.0)


def at(items, target):
  """Lane line y at a given distance, linear between the model's points."""
  xs = [float(v) for v in items.x]
  ys = [float(v) for v in items.y]
  for i in range(len(xs) - 1):
    if xs[i] <= target <= xs[i + 1] and xs[i + 1] > xs[i]:
      f = (target - xs[i]) / (xs[i + 1] - xs[i])
      return ys[i] + f * (ys[i + 1] - ys[i])
  return None


offsets = {d: [] for d in LOOK_M}
speeds = []
steer = []
t = []
prob_left = []
prob_right = []
missing = 0
frames = 0

for msg in LogReader(PATH):
  w = msg.which()
  if w == "carState":
    speeds.append(float(msg.carState.vEgo))
    steer.append(float(msg.carState.steeringAngleDeg))
  elif w == "modelV2":
    frames += 1
    pl = [float(p) for p in msg.modelV2.laneLineProbs]
    prob_left.append(pl[1])
    prob_right.append(pl[2])
    if len(msg.modelV2.laneLines) > 2:
      left, right = msg.modelV2.laneLines[1], msg.modelV2.laneLines[2]
      for d in LOOK_M:
        a, b = at(left, d), at(right, d)
        if a is not None and b is not None:
          offsets[d].append((a + b) / 2.0)
        else:
          missing += 1

if frames == 0:
  print("no model frames in %s" % PATH)
  raise SystemExit(0)

v = statistics.median(speeds) if speeds else 0.0
print("frames %d, median speed %.1f m/s (%.0f km/h)" % (frames, v, v * 3.6))
print()
print("lane-centre offset from the car (metres; signed, + is lane centre to the right):")
for d in LOOK_M:
  s = offsets[d]
  if not s:
    print("  %4.0f m : no data" % d)
    continue
  rms = math.sqrt(sum(x * x for x in s) / len(s))
  print("  %4.0f m : median %+.3f  rms %.3f  p10 %+.3f  p90 %+.3f  (n=%d)"
        % (d, statistics.median(s), rms, sorted(s)[len(s) // 10], sorted(s)[9 * len(s) // 10], len(s)))

print()
print("lane pair confidence: left median %.2f, right median %.2f"
      % (statistics.median(prob_left), statistics.median(prob_right)))
faint = sum(1 for a, b in zip(prob_left, prob_right) if min(a, b) < 0.5)
print("frames where either line is below the car's own 0.5 threshold: %d of %d (%.0f%%)"
      % (faint, frames, 100.0 * faint / frames))
print("distance lookups with no lane data at all: %d" % missing)

if len(steer) > 60:
  d1 = [b - a for a, b in zip(steer, steer[1:])]
  act = statistics.median([abs(x) for x in d1])
  print()
  print("steering: median |change between frames| %.3f deg (a hunting wheel shows a big number here)"
        % act)
  if v > 5.0 and len(steer) > 400:
    # crude movement spectrum in the 0.2-2 Hz band where centring hunting lives
    import cmath
    n = 512
    seg = steer[:n]
    mean = sum(seg) / n
    seg = [x - mean for x in seg]
    best = (0.0, 0.0)
    for k in range(2, 40):
      acc = sum(seg[i] * cmath.exp(-2j * math.pi * k * i / n).real for i in range(n))
      if abs(acc) > best[1]:
        best = (k / n * 100.0, abs(acc))     # ~100 Hz sample rate assumed
    print("strongest steering oscillation near %.2f Hz (amplitude %.2f)" % best)
