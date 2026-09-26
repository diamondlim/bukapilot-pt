#!/usr/bin/env python3
"""Are both road cameras alive, and does covering one show up in its exposure?

Read-only: subscribes to the two camera states that already exist. Nothing is sent.
"""
import sys
import time

sys.path.insert(0, "/data/openpilot")
import cereal.messaging as messaging  # noqa: E402

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
sm = messaging.SubMaster(["roadCameraState", "wideRoadCameraState"])

first = {}
last = {}
deadline = time.time() + SECONDS
while time.time() < deadline:
  sm.update(100)
  for name in ("roadCameraState", "wideRoadCameraState"):
    if not sm.updated[name]:
      continue
    s = sm[name]
    fid = int(s.frameId)
    first.setdefault(name, (fid, float(s.exposureValPercent), float(s.gain)))
    last[name] = (fid, float(s.exposureValPercent), float(s.gain))

for name in ("roadCameraState", "wideRoadCameraState"):
  if name not in last:
    print("%-22s : NO FRAMES" % name)
    continue
  f0, e0, g0 = first[name]
  f1, e1, g1 = last[name]
  print("%-22s : %4d frames in %.1f s (%.1f fps), exposure %.0f%% -> %.0f%%, gain %.1f -> %.1f"
        % (name, f1 - f0, SECONDS, (f1 - f0) / SECONDS, e0, e1, g0, g1))
