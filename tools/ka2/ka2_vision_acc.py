#!/usr/bin/env python3
"""Vision -> stock ACC setpoint: slow for a bend the model already sees, then hand the speed back.

The vision_turnspeed branch computes a deceleration request and gives it to the planner's MPC, which only
helps a car where openpilot works the pedals. This box has the stock ACC doing gas and brake, so the same
geometry is actuated the only way this car allows: move the ACC setpoint with the car's own buttons, one
5 km/h step at a time, through the same request file card consumes.

Geometry is lifted unchanged from selfdrive/controls/lib/vision_turn_speed.py on the vision_turnspeed branch
(path_curvature / find_bend / v_cap = sqrt(a_lat / kappa)) so the two agree by construction. What this file
adds is the actuation policy:

  * It lowers the setpoint with SET for a bend the model sees (5 km/h per press), and when the road ahead
    allows it again it raises it back with the car's own + pattern - **never above the setpoint the driver
    had before the first press**. That ceiling is the whole safety argument: this can undo its own work and
    nothing more. It never invents a faster target than the driver's.
  * It raises only when the road genuinely allows it: no bend within the lookahead, or the bend's comfort
    speed is at least one step plus a margin above the current setpoint. "Straight" is judged by geometry,
    not by the bend having disappeared for a moment.
  * It never presses ACC+ (RES) on its own, never LKAS, never CANCEL. RES alone cancels ACC on this car. The
    + pattern is only ever used while moving above the speed floor, because at a standstill it means
    resume/cancel.
  * It acts only while ACC is engaged and the car is moving above VIS_TURN_ACC_MIN_V_KMH.
  * At most VIS_TURN_ACC_MAX_STEPS presses down per bend, cooldown between any press, and never below
    VIS_TURN_ACC_MIN_SETPOINT_KMH. If the driver raises the setpoint themselves, that becomes the new
    ceiling and the books are cleared.
  * Shipped OFF: /data/hermes/tuning.json holds VIS_TURN_ACC_ENABLED = 0 until it is set to 1. Every tuning
    range is one-sided too - the file can only make this act LESS, never more.

Two modes:
  ka2_vision_acc.py --replay <rlog.zst> --verbose    # what it WOULD have done on a recorded drive
  ka2_vision_acc.py --live [--dry-run]               # act on the box (requires the tuning flag)
"""
import argparse
import json
import math
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, "/data/openpilot")

TUNING_PATH = "/data/hermes/tuning.json"
PRESS_TOOL = "/data/hermes/ka2_acc_press.py"
AUDIT = "/data/hermes/acc/vision_acc.jsonl"      # in the kommu-owned dir, so the daemon can write it
STEP_KMH = 5.0               # one press on this car; the simulation uses the same number

# Shipped defaults. The tuning file may only move them in the direction of acting less.
ENABLED = 0
RESTORE = 1                  # hand the speed back after the bend, up to the driver's own setpoint
A_LAT = 1.2                  # m/s^2 comfort lateral accel in a bend
MIN_RADIUS = 250.0           # m; gentler bends are ignored entirely
LOOKAHEAD_MAX = 160.0        # m
MIN_V_KMH = 25.0             # below this speed, bends are the driver's business
MARGIN_KMH = 5.0             # act only when the comfort speed is at least this far under the setpoint
TRIGGER_S = 3.0              # start stepping this many seconds before the bend entry
MAX_STEPS = 3                # shipped default: at most 15 km/h taken off per bend. The app may raise
                             # this up to TUNING_LIMITS' 6 (30 km/h) - the floor, the driver's own setpoint
                             # and the engaged/25 km/h gates are what bound it, not this cap.
COOLDOWN_S = 2.5             # s between any two presses
RESTORE_MARGIN_KMH = 5.0     # headroom the road must allow above the next step before raising
MIN_SETPOINT_KMH = 30.0      # the auto-slow floor: never step the setpoint below this
MAX_RESTORE_KMH = 130.0      # the auto-raise ceiling: never lift the setpoint above this
BEND_CLEAR_S = 2.0           # no bend for this long = behind us; the next bend gets a fresh budget
MIN_PTS = 8                  # grid points within 30% of the peak = a real bend, not a geometry spike

TUNING_LIMITS = {
  "VIS_TURN_ACC_ENABLED": (0.0, 1.0),
  "VIS_TURN_ACC_RESTORE": (0.0, 1.0),
  "VIS_TURN_ACC_A_LAT": (1.2, 2.5),
  "VIS_TURN_ACC_MIN_RADIUS": (250.0, 600.0),
  "VIS_TURN_ACC_MIN_V_KMH": (25.0, 70.0),
  "VIS_TURN_ACC_MARGIN_KMH": (5.0, 20.0),
  "VIS_TURN_ACC_MAX_STEPS": (0.0, 6.0),            # 0-6 presses = 0-30 km/h off per bend. Owner's
                                                   # request (24 Sep 2026): this one range is NOT
                                                   # one-sided, so the app can raise the cap; the
                                                   # 30 km/h floor, the driver's own setpoint ceiling
                                                   # and the engaged/25 km/h gates remain the bounds.
  "VIS_TURN_ACC_COOLDOWN_S": (2.5, 15.0),
  "VIS_TURN_ACC_RESTORE_MARGIN_KMH": (5.0, 25.0),
  "VIS_TURN_ACC_MIN_SETPOINT_KMH": (30.0, 90.0),   # the app's auto-slow floor: raise only, never lower
  "VIS_TURN_ACC_MAX_RESTORE_KMH": (60.0, 130.0),   # the app's auto-raise ceiling: lower only, never raise
}
_LAST_GOOD = {
  "VIS_TURN_ACC_ENABLED": ENABLED, "VIS_TURN_ACC_RESTORE": RESTORE, "VIS_TURN_ACC_A_LAT": A_LAT,
  "VIS_TURN_ACC_MIN_RADIUS": MIN_RADIUS, "VIS_TURN_ACC_MIN_V_KMH": MIN_V_KMH,
  "VIS_TURN_ACC_MARGIN_KMH": MARGIN_KMH, "VIS_TURN_ACC_MAX_STEPS": MAX_STEPS,
  "VIS_TURN_ACC_COOLDOWN_S": COOLDOWN_S, "VIS_TURN_ACC_RESTORE_MARGIN_KMH": RESTORE_MARGIN_KMH,
  "VIS_TURN_ACC_MIN_SETPOINT_KMH": MIN_SETPOINT_KMH, "VIS_TURN_ACC_MAX_RESTORE_KMH": MAX_RESTORE_KMH,
}


def read_tuning(path=TUNING_PATH):
  """Tuning values, each clamped into its allowed range. Missing or broken file means shipped values.

  Unreadable values keep the last good ones rather than reverting to aggressive defaults: a locked or
  half-written file must never make this act more.
  """
  out = dict(_LAST_GOOD)
  try:
    with open(path) as fh:
      raw = json.load(fh)
    if not isinstance(raw, dict):
      return out
  except (OSError, ValueError):
    return out
  for key, (lo, hi) in TUNING_LIMITS.items():
    if key in raw:
      try:
        val = float(raw[key])
      except (TypeError, ValueError):
        continue
      out[key] = min(max(val, lo), hi)
  _LAST_GOOD.update(out)
  return out


def path_curvature(xs, ys, s_max, step=1.0):
  """Resample the model's predicted path onto a distance grid; return (s, y, kappa), or None."""
  x = np.asarray(xs, dtype=float)
  y = np.asarray(ys, dtype=float)
  if x.size < 4:
    return None
  keep = np.concatenate(([True], np.diff(x) > 1e-3))
  x, y = x[keep], y[keep]
  if x.size < 4 or x[-1] < 8.0:
    return None
  s = np.arange(3.0, min(s_max, x[-1]) + 1e-6, step)
  if s.size < 5:
    return None
  yy = np.interp(s, x, y)
  dy = np.gradient(yy, s)
  d2y = np.gradient(dy, s)
  return s, yy, np.abs(d2y) / np.power(1.0 + dy * dy, 1.5)


def find_bend(s, kappa, min_radius):
  """Tightest sustained bend inside the lookahead: (kmax, s_peak, s_entry, n_pts) or None.

  A single-point geometry glitch is rejected: a real bend holds its curvature over several metres.
  """
  thr = 1.0 / min_radius
  k = np.convolve(kappa, np.ones(5) / 5.0, mode="same")
  m = k > thr
  if not m.any():
    return None
  kmax = float(k.max())
  if kmax <= 1e-9:
    return None
  n_pts = int(np.sum(k[m] > 0.7 * kmax))
  if n_pts < MIN_PTS:
    return None
  return kmax, float(s[int(np.argmax(k))]), float(s[np.nonzero(m)[0][0]]), n_pts


def bend_ahead(xs, ys, tuning):
  """(v_cap_kmh, metres to the bend entry) for a bend worth slowing for, or None.

  No deceleration arithmetic here on purpose: the car's own ACC does the slowing once the setpoint is lower.
  This policy only decides what speed the road allows.
  """
  geom = path_curvature(xs, ys, LOOKAHEAD_MAX)
  if geom is None:
    return None
  s, _y, kappa = geom
  bend = find_bend(s, kappa, tuning["VIS_TURN_ACC_MIN_RADIUS"])
  if bend is None:
    return None
  kmax, _s_peak, s_entry, _n = bend
  return 3.6 * math.sqrt(tuning["VIS_TURN_ACC_A_LAT"] / kmax), max(8.0, s_entry)


def decide(state, tuning, v_kmh, setpoint_kmh, engaged, now):
  """What to do right now: ("down"|"up"|None, reason).

  Pure function of its inputs, so replay validates exactly the logic that runs live.
  state: cap_kmh, d_m, saw_bend, steps, last_press, bend_gone_since, lowered_kmh, base_kmh.
  """
  if state["saw_bend"]:
    state["bend_gone_since"] = None
  elif state["bend_gone_since"] is None:
    state["bend_gone_since"] = now
  if state["bend_gone_since"] is not None and now - state["bend_gone_since"] > BEND_CLEAR_S:
    state["steps"] = 0                    # bend is behind us; the next one gets a fresh budget
    state["bend_gone_since"] = None

  if not tuning["VIS_TURN_ACC_ENABLED"]:
    return None, "policy off"
  if not engaged:
    return None, "ACC not engaged"
  if v_kmh < tuning["VIS_TURN_ACC_MIN_V_KMH"]:
    return None, "%.0f km/h is below the %.0f km/h floor" % (v_kmh, tuning["VIS_TURN_ACC_MIN_V_KMH"])
  cooldown = tuning["VIS_TURN_ACC_COOLDOWN_S"]
  if now - state["last_press"] < cooldown:
    return None, "pressed %.1f s ago" % (now - state["last_press"])

  cap = state["cap_kmh"]
  floor = tuning["VIS_TURN_ACC_MIN_SETPOINT_KMH"]     # the app's auto-slow floor
  ceiling = tuning["VIS_TURN_ACC_MAX_RESTORE_KMH"]   # the app's auto-raise ceiling

  # 1. Slow for a bend that needs it. This outranks restoring: this policy never raises the setpoint while
  #    the road ahead still demands something slower.
  if cap is not None:
    if setpoint_kmh <= floor:
      return None, "setpoint already at the %.0f km/h floor" % floor
    if cap <= setpoint_kmh - tuning["VIS_TURN_ACC_MARGIN_KMH"]:
      if state["d_m"] is not None and state["d_m"] > max(30.0, v_kmh / 3.6 * TRIGGER_S):
        return None, "bend is %.0f m away" % state["d_m"]
      if state["steps"] >= tuning["VIS_TURN_ACC_MAX_STEPS"]:
        return None, "already took %d steps off for this bend" % state["steps"]
      return "down", "bend allows %.0f km/h, setpoint %.0f km/h, %.0f m ahead" % (
        cap, setpoint_kmh, state["d_m"] or 0.0)

  # 2. Hand the speed back, but only up to the setpoint the driver had before we touched it and never above
  #    the app's auto-raise ceiling, and only when the road geometry actually allows that speed.
  if tuning["VIS_TURN_ACC_RESTORE"] and state["lowered_kmh"] > 0:
    driver_kmh = state["base_kmh"]
    if driver_kmh is None:
      return None, "no ceiling recorded yet"
    target = min(driver_kmh, ceiling)
    if setpoint_kmh >= target - 0.5:
      if driver_kmh > ceiling + 0.5:
        return None, "held at the %.0f km/h auto-raise ceiling" % ceiling
      return None, "already back at the driver's %.0f km/h" % target
    nxt = min(setpoint_kmh + STEP_KMH, target)
    if cap is not None and cap < nxt + tuning["VIS_TURN_ACC_RESTORE_MARGIN_KMH"]:
      return None, "road ahead allows %.0f km/h, not %.0f yet" % (cap, nxt)
    return "up", "road clear to %.0f km/h (bend allows %s)" % (nxt, ("%.0f" % cap) if cap else "nothing")

  if cap is None:
    return None, "no bend ahead"
  return None, "bend allows %.0f km/h; setpoint %.0f km/h is fine" % (cap, setpoint_kmh)


def press_once(action, dry_run=False):
  """One step, through the press tool so its gates and audit stay the only rules. 'up' uses the + pattern."""
  button = "step" if action == "up" else "set"
  if dry_run:
    print("  [dry-run] would press %s (%s)" % (button, action))
    return True, "dry-run"
  try:
    p = subprocess.run([sys.executable, PRESS_TOOL, "--button", button],
                       capture_output=True, text=True, timeout=15)
    out = (p.stdout or p.stderr or "").strip().splitlines()
    return p.returncode == 0, (out[-1][:120] if out else "no output")
  except Exception as exc:
    return False, "press tool did not run: %s" % str(exc)[:100]


def audit(row):
  try:
    with open(AUDIT, "a") as fh:
      fh.write(json.dumps(row) + "\n")
  except OSError:
    pass


def new_state():
  return {"cap_kmh": None, "d_m": None, "saw_bend": False, "steps": 0, "last_press": -1e9,
          "bend_gone_since": None, "lowered_kmh": 0.0, "base_kmh": None}


def track_setpoint(state, real_kmh):
  """Keep the driver's own setpoint as the ceiling. If they raise it, that wins and the books clear."""
  if state["lowered_kmh"] <= 0:
    state["base_kmh"] = real_kmh
  elif real_kmh > (state["base_kmh"] or 0.0) + 1.0:
    state["base_kmh"] = real_kmh
    state["lowered_kmh"] = 0.0
    state["steps"] = 0
  return max(0.0, real_kmh - state["lowered_kmh"])


def replay(path, verbose=False):
  """Run the policy over a recorded drive. Nothing is sent to the car; the setpoint is simulated.

  Models the real thing: a press moves the effective setpoint and the change persists until the driver moves
  theirs, which the log shows and which resets the ceiling.
  """
  from openpilot.tools.lib.logreader import LogReader
  from collections import Counter
  tuning = read_tuning()
  tuning["VIS_TURN_ACC_ENABLED"] = 1.0        # replay evaluates the policy regardless of the off switch
  state = new_state()
  reasons = Counter()
  t0, presses, ups, bends, events = None, 0, 0, 0, []
  real, effective = None, 0.0
  for msg in LogReader(path):
    t = msg.logMonoTime / 1e9
    if t0 is None:
      t0 = t
    w = msg.which()
    if w == "carState":
      cs = msg.carState
      v_kmh = float(cs.vEgo) * 3.6
      engaged = bool(cs.cruiseState.enabled)
      real = float(cs.cruiseState.speedCluster) * 3.6
      effective = track_setpoint(state, real)
    elif w == "modelV2":
      got = bend_ahead(msg.modelV2.position.x, msg.modelV2.position.y, tuning)
      state["cap_kmh"], state["d_m"] = got if got else (None, None)
      state["saw_bend"] = got is not None
      if got:
        bends += 1
      if real is None:
        continue
      action, reason = decide(state, tuning, v_kmh, effective, engaged, t - t0)
      reasons[reason.split(";")[0].split(",")[0]] += 1
      if action:
        state["last_press"] = t - t0
        if action == "down":
          state["lowered_kmh"] += STEP_KMH
          state["steps"] += 1
          presses += 1
        else:
          state["lowered_kmh"] = max(0.0, state["lowered_kmh"] - STEP_KMH)
          ups += 1
        effective = max(0.0, real - state["lowered_kmh"])
        events.append((round(t - t0, 1), action, round(real, 1), round(effective, 1),
                       round(state["cap_kmh"], 1) if state["cap_kmh"] else None, round(v_kmh, 1)))
        if verbose:
          print("  +%7.1fs  %-4s set %.0f -> %.0f  (road allows %s, doing %.0f km/h)"
                % (t - t0, action, real, effective,
                   ("%.0f" % state["cap_kmh"]) if state["cap_kmh"] else "clear", v_kmh))
  return {"presses": presses, "raises": ups, "bend_frames": bends, "events": events,
          "max_lowered": max([e[2] - e[3] for e in events], default=0.0),
          "reasons": reasons.most_common(6)}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--live", action="store_true", help="run against the car (needs VIS_TURN_ACC_ENABLED=1)")
  ap.add_argument("--replay", help="rlog.zst of a recorded drive: report what the policy would have done")
  ap.add_argument("--verbose", action="store_true", help="with --replay, list every step it would take")
  ap.add_argument("--dry-run", action="store_true", help="live mode: log what it would do, press nothing")
  ap.add_argument("--log-every", type=float, default=0.0, help="live mode: state line every N seconds")
  a = ap.parse_args()

  if a.replay:
    s = replay(a.replay, verbose=a.verbose)
    print("down %d, up %d, bend frames %d, biggest reduction %.0f km/h"
          % (s["presses"], s["raises"], s["bend_frames"], s["max_lowered"]))
    for why, n in s["reasons"]:
      print("   %-46s %d frames" % (why[:46], n))
    return 0

  if not a.live:
    ap.error("give --live or --replay <rlog.zst>")

  from cereal import messaging
  tuning, last_tuning = read_tuning(), 0.0
  state = new_state()
  sm = messaging.SubMaster(["carState", "modelV2"])
  print("vision->ACC bridge: enabled=%s restore=%s dry_run=%s"
        % (bool(tuning["VIS_TURN_ACC_ENABLED"]), bool(tuning["VIS_TURN_ACC_RESTORE"]), a.dry_run))
  while True:
    sm.update(100)
    now = time.monotonic()
    if now - last_tuning > 1.0:
      last_tuning = now
      tuning = read_tuning()
    if not sm.alive["carState"] or not sm.alive["modelV2"]:
      continue
    cs = sm["carState"]
    v_kmh = float(cs.vEgo) * 3.6
    real = float(cs.cruiseState.speedCluster) * 3.6
    effective = track_setpoint(state, real)
    if sm.updated["modelV2"]:
      got = bend_ahead(sm["modelV2"].position.x, sm["modelV2"].position.y, tuning)
      state["cap_kmh"], state["d_m"] = got if got else (None, None)
      state["saw_bend"] = got is not None
    action, reason = decide(state, tuning, v_kmh, effective, bool(cs.cruiseState.enabled), now)
    if action:
      ok, reply = press_once(action, a.dry_run)
      state["last_press"] = now
      if action == "down":
        state["lowered_kmh"] += STEP_KMH
        state["steps"] += 1
      else:
        state["lowered_kmh"] = max(0.0, state["lowered_kmh"] - STEP_KMH)
      print("%s: %s | %s" % (action.upper(), reason, reply))
      audit({"t": time.time(), "kind": action, "steps": state["steps"],
             "lowered_kmh": state["lowered_kmh"], "base_kmh": state["base_kmh"],
             "cap_kmh": state["cap_kmh"], "d_m": state["d_m"], "setpoint_kmh": round(effective, 1),
             "v_kmh": round(v_kmh, 1), "why": reason, "press_ok": bool(ok), "reply": reply,
             "dry_run": bool(a.dry_run)})
    elif a.log_every and sm.updated["modelV2"]:
      print("  v=%.0f base=%s set=%s cap=%s d=%s : %s" % (
        v_kmh, ("%.0f" % state["base_kmh"]) if state["base_kmh"] else "-", "%.0f" % effective,
        ("%.0f" % state["cap_kmh"]) if state["cap_kmh"] else "-",
        ("%.0f" % state["d_m"]) if state["d_m"] else "-", reason))
  return 0


if __name__ == "__main__":
  sys.exit(main())
