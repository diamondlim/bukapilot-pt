#!/usr/bin/env python3
"""Show what position the box actually publishes - the honest check after bring-up.

Subscribe to gpsLocationExternal for a while and report what arrived: whether anything did, the
coordinates, fix state, satellites and the age of the fix. Nothing here is inferred from the publisher's
own log; a silent result is reported as silence.
"""
import argparse
import sys
import time

import cereal.messaging as messaging


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    sock = messaging.sub_sock("gpsLocationExternal", timeout=2000)
    seen = 0
    best = None
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        for msg in messaging.drain_sock(sock, wait_for_one=True):
            if not msg.which() == "gpsLocationExternal":
                continue
            gps = msg.gpsLocationExternal
            seen += 1
            if gps.hasFix and (best is None or gps.satelliteCount >= best.satelliteCount):
                best = gps
    if seen == 0:
        print("no gpsLocationExternal message in %.0fs - nothing is publishing a position" % args.seconds)
        return 1
    if best is None:
        print("%d messages in %.0fs, none with a fix (module powered but no sky view, or no antenna)"
              % (seen, args.seconds))
        return 2
    age_s = (time.time() * 1000 - best.unixTimestampMillis) / 1000.0
    print("%d messages | fix: %.6f, %.6f  alt %.1f m | %d sats | hdop %.1f (%.0f m) | speed %.1f m/s | "
          "fix time age %.0f s" % (seen, best.latitude, best.longitude, best.altitude,
                                   best.satelliteCount, best.horizontalAccuracy / 5.0,
                                   best.horizontalAccuracy, best.speed, age_s))
    return 0


if __name__ == "__main__":
    sys.exit(main())
