#!/usr/bin/env python3
"""Settle the question: does this car honour a spoofed ACC set-speed press?

Run this on the box while sitting in the car with ACC switched on (ideally engaged - press the stalk's
SET so ACC is active). It watches the car's own SET_SPEED for a while, presses one button through the
normal app path at a known moment, and prints the set speed before/after, second by second. If the number
moves when the press happens, the car obeys the spoofed frame; if it does not, the press is being ignored
and no amount of app work will change that.

  /usr/local/venv/bin/python3 /data/hermes/ka2_acc_watch_press.py --button res --delay 6 --watch 16

Careful: this presses a real button on a real car. Use it with the car stationary or on an empty road.
"""
import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, "/data/hermes")
sys.path.insert(0, "/data/openpilot")


def read_set_speed(timeout_s=1.0):
    """(available, enabled, set_speed_kph, vEgo_kph) or Nones if carState is unavailable."""
    from cereal import messaging
    sm = messaging.SubMaster(["carState"])
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        sm.update(100)
        if sm.updated["carState"]:
            cs = sm["carState"]
            return (bool(cs.cruiseState.available), bool(cs.cruiseState.enabled),
                    float(cs.cruiseState.speedCluster) * 3.6, float(cs.vEgo) * 3.6, str(cs.gearShifter))
    return None, None, None, None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--button", default="res", choices=sorted(("res", "set", "cancel", "inc_dist", "dec_dist")))
    ap.add_argument("--delay", type=float, default=6.0, help="seconds to watch before pressing")
    ap.add_argument("--watch", type=float, default=16.0, help="total seconds to watch")
    a = ap.parse_args()

    tool = "/data/hermes/ka2_acc_press.py"
    t0 = time.time()
    seen = []
    pressed = False
    while time.time() - t0 < a.watch:
        elapsed = time.time() - t0
        if not pressed and elapsed >= a.delay:
            before = read_set_speed()
            print("  %5.1fs  pressing %s now (set=%.1f kph before, gear=%s engaged=%s)" % (elapsed, a.button, before[2], before[4], before[1]))
            out = subprocess.run([sys.executable, tool, "--button", a.button],
                                 capture_output=True, text=True, timeout=30)
            print("  %5.1fs  tool said: %s" % (time.time() - t0, (out.stdout or out.stderr).strip()))
            pressed = True
        avail, enabled, set_kph, vego, gear = read_set_speed()
        if set_kph is None:
            print("  %5.1fs  carState unavailable" % elapsed)
        else:
            line = "%5.1fs  set=%5.1f kph  vEgo=%5.1f  gear=%-5s ACC avail=%s engaged=%s" % (
                elapsed, set_kph, vego, gear, avail, enabled)
            print(line)
            seen.append(set_kph)
        time.sleep(1.0)

    if pressed and seen:
        uniq = []
        for v in seen:
            if not uniq or abs(v - uniq[-1]) > 0.05:
                uniq.append(v)
        if len(uniq) > 1:
            print("\nVERDICT: the set speed moved through %s - the car DID honour the press." %
                  " -> ".join("%.1f" % v for v in uniq))
        else:
            print("\nVERDICT: set speed stayed at %.1f kph - the car ignored the press in this state."
                  % uniq[0])
            print("Try again with ACC engaged (engages only above walking pace on most BYDs), then on a")
            print("road: if it still does not move, the spoofed button is not accepted at all.")


if __name__ == "__main__":
    main()
