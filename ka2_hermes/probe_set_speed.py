#!/usr/bin/env python3
"""Why is the set speed wrong, and why does it lag?

Reads the published pose file and the car's own carState side by side for 20 seconds, so the answer is
measured rather than reasoned:

  * does the published set speed match what the car reports, and does it react at once?
  * what does the raw HUD value look like, against the fork's own normalisation (it divides by
    HUD_MULTIPLIER and clamps the cluster value at 30 km/h)?
  * how big is the pose line, and what bitrate does it imply over Bluetooth at the published rate?
"""
import json
import sys
import time

sys.path.insert(0, "/data/openpilot")
import cereal.messaging as messaging  # noqa: E402

POSE = "/dev/shm/ka2_pose.json"
RATE_HZ = 10.0

sm = messaging.SubMaster(["carState"], ignore_alive=["carState"])

print("t      | pose: v      set      bytes | carState: vEgo    cruise    cluster   | match")
print("-" * 96)
rows = []
start = time.time()
last_pose = None
while time.time() - start < 20:
    sm.update(200)
    try:
        raw = open(POSE).read()
        pose = json.loads(raw)
    except Exception as exc:
        print("pose unreadable:", exc)
        continue
    if sm.updated["carState"]:
        cs = sm["carState"]
        cluster = float(cs.cruiseState.speedCluster)
        cruise = float(cs.cruiseState.speed)
        vego = float(cs.vEgo)
        pose_set = float(pose.get("set", 0.0))
        match = "yes" if abs(pose_set - cruise) < 1e-6 else "NO"
        rows.append((len(raw), pose_set, cruise, cluster, vego))
        print("%6.1f | %.2f m/s  %6.2f  %6d | %.3f m/s  %6.2f  %6.2f | %s" % (
            time.time() - start, pose.get("v", 0.0), pose_set, len(raw), vego, cruise, cluster, match))
    time.sleep(0.2)

if rows:
    sizes = [r[0] for r in rows]
    print()
    print("pose line: median %d bytes, max %d" % (sorted(sizes)[len(sizes) // 2], max(sizes)))
    print("implied at %.0f Hz: %.1f KB/s  (%.0f kbps) - Bluetooth SPP manages roughly 200-700 kbps" % (
        RATE_HZ, RATE_HZ * sorted(sizes)[len(sizes) // 2] / 1024.0,
        RATE_HZ * sorted(sizes)[len(sizes) // 2] * 8 / 1000.0))
    mismatches = [r for r in rows if abs(r[1] - r[2]) > 1e-6]
    print("set speed matches carState in %d of %d samples" % (len(rows) - len(mismatches), len(rows)))
    clusters = sorted(set(round(r[3], 2) for r in rows))
    cruises = sorted(set(round(r[2], 2) for r in rows))
    print("distinct cluster values seen: %s" % clusters)
    print("distinct cruiseState.speed values: %s" % cruises)
    if cruises and cruises[0] > 0:
        print("ratio cluster/cruise: %.3f  (the fork divides by its HUD_MULTIPLIER of 1.12)" % (
            clusters[0] / cruises[0]))
