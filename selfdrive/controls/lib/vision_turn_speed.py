#!/usr/bin/env python3
"""Vision turn-speed: slow the car early for a bend the model can already see.

Why this exists: this fork's longitudinal planner has no curve handling at all. Its MPC
follows the lead it is given and the cruise set speed, so a bend the car cannot see into is
taken at whatever the driver/cruise was doing. The model, however, publishes a ten-second
predicted path on every car, every frame - so a bend IS known before the car reaches it.
This module turns that path into a deceleration request, early enough to be comfortable.

Design rule (the whole point of "early"): do not cap the speed when the bend arrives - work
out the deceleration needed NOW to be at the bend's comfort speed by the time the bend
starts. For a bend whose entry is at distance d and whose comfort speed is
v_cap = sqrt(a_lat / kappa), that is

    a_req = (v_cap^2 - v^2) / (2 * d)      (negative: a deceleration)

clamped to a comfort decel. Inside the bend d collapses to its floor, so the request
degenerates into holding v_cap - no discontinuity at the entry.

Everything is a pure function of (path, v_ego) plus one hold timer, so it is unit-testable
off the car. The tunables are read live (never captured as default arguments) so the JSON
tuning file can move them while driving - see `TUNING_LIMITS`.

Off switch: VISION_TURN_SPEED_ENABLED = 0 restores stock behaviour exactly - the module
returns no request and the planner's own output is untouched. That is the shipped default.
"""
import json
import math

import numpy as np

# ---------------------------------------------------------------------------
# tuning; every value has an inert state, and every range below is one-sided
# ---------------------------------------------------------------------------
VISION_TURN_SPEED_ENABLED = 0        # 0 = off (stock), 1 = on. Shipped OFF.
VISION_TURN_A_LAT = 1.2             # m/s^2 target lateral accel in a bend (comfort, not limit)
VISION_TURN_A_DEC_MAX = 1.5         # m/s^2 most decel this policy may ask for on its own
VISION_TURN_MIN_RADIUS = 250.0      # m; bends gentler than this are not slowed for at all
VISION_TURN_LOOKAHEAD_MAX = 160.0   # m; how far down the predicted path to look
VISION_TURN_MIN_D = 8.0             # m; distance floor, so a_req cannot explode near the bend
VISION_TURN_MIN_PTS = 8             # grid points (1 m) within 30% of the peak = a real bend,
                                    # not a one-frame geometry spike
VISION_TURN_HOLD_S = 0.5            # s; keep an in-progress request through one noisy frame
PATH_STEP = 1.0                     # m; resampling grid for the predicted path

# Same file controlsd reads for the lateral knobs. Ranges are one-sided on purpose: the file
# may move this policy toward the gentler side (less decel, a tighter radius before it acts,
# less lateral accel in the bend) or switch it off, but never make it more aggressive than the
# code committed here - turning a car's speed behaviour *up* belongs in a commit with a drive
# behind it, not in a slider.
TUNING_PATH = "/data/hermes/tuning.json"
TUNING_LIMITS = {
  "VIS_TURN_ENABLED": (0.0, 1.0),          # 0 = off (stock)
  "VIS_TURN_A_LAT": (0.8, 1.2),            # the shipped 1.2 m/s^2 is the ceiling
  "VIS_TURN_A_DEC_MAX": (0.5, 1.5),        # the shipped 1.5 m/s^2 is the ceiling
  "VIS_TURN_MIN_RADIUS": (250.0, 500.0),   # act on tighter bends only
  "VIS_TURN_HOLD_S": (0.0, 0.5),           # less memory; 0.0 = stateless
}


def read_tuning(path=TUNING_PATH, limits=TUNING_LIMITS):
  """{name: clamped value} for the keys the tuning file carries, else {}.

  Never raises: an absent, malformed or out-of-range file leaves the shipped constants in
  place. Deleting the file is the off switch, not setting a flag.
  """
  out = {}
  try:
    with open(path) as fh:
      data = json.load(fh)
  except Exception:
    return out
  if not isinstance(data, dict):
    return out
  for name, (low, high) in limits.items():
    if name not in data:
      continue
    try:
      value = float(data[name])
    except (TypeError, ValueError):
      continue
    if not math.isfinite(value):
      continue
    out[name] = float(np.clip(value, low, high))
  return out


# the shipped value of every tunable, captured once so a key that disappears from the file
# restores the constant rather than freezing the last tuned value
TUNING_BASE = {
  "VIS_TURN_ENABLED": VISION_TURN_SPEED_ENABLED,
  "VIS_TURN_A_LAT": VISION_TURN_A_LAT,
  "VIS_TURN_A_DEC_MAX": VISION_TURN_A_DEC_MAX,
  "VIS_TURN_MIN_RADIUS": VISION_TURN_MIN_RADIUS,
  "VIS_TURN_HOLD_S": VISION_TURN_HOLD_S,
}


def apply_tuning(tuning):
  """Push tuning values into the module globals this policy reads each call."""
  globals()["VISION_TURN_SPEED_ENABLED"] = int(tuning.get("VIS_TURN_ENABLED", TUNING_BASE["VIS_TURN_ENABLED"]) >= 0.5)
  globals()["VISION_TURN_A_LAT"] = tuning.get("VIS_TURN_A_LAT", TUNING_BASE["VIS_TURN_A_LAT"])
  globals()["VISION_TURN_A_DEC_MAX"] = tuning.get("VIS_TURN_A_DEC_MAX", TUNING_BASE["VIS_TURN_A_DEC_MAX"])
  globals()["VISION_TURN_MIN_RADIUS"] = tuning.get("VIS_TURN_MIN_RADIUS", TUNING_BASE["VIS_TURN_MIN_RADIUS"])
  globals()["VISION_TURN_HOLD_S"] = tuning.get("VIS_TURN_HOLD_S", TUNING_BASE["VIS_TURN_HOLD_S"])


def path_curvature(xs, ys, s_max, step=PATH_STEP):
  """Resample the model's predicted path onto a distance grid and return (s, y, kappa).

  xs/ys are the model's predicted x (forward, m) and y (lateral, m) as an ungridded set of
  points, so resample before differentiating. kappa is |y''| / (1 + y'^2)^1.5 - the 2-D path
  curvature, not just y''.
  """
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
  kappa = np.abs(d2y) / np.power(1.0 + dy * dy, 1.5)
  return s, yy, kappa


def find_bend(s, kappa, min_radius=None):
  """The bend that matters: tightest sustained curvature inside the lookahead.

  Returns (kmax, s_peak, s_entry, n_pts) or None. A single-point spike is rejected - a real
  bend holds its curvature over several metres, so demand at least VISION_TURN_MIN_PTS grid
  points within 30% of the peak. s_entry is where the bend first crosses the threshold: the
  point the car must already be slowed down for.
  """
  min_radius = VISION_TURN_MIN_RADIUS if min_radius is None else min_radius
  thr = 1.0 / min_radius
  # smooth over 5 m before looking for a peak: a single-point geometry glitch would
  # otherwise set kmax, and no real bend would then come within 30% of it
  k = np.convolve(kappa, np.ones(5) / 5.0, mode="same")
  m = k > thr
  if not m.any():
    return None
  kmax = float(k.max())
  if kmax <= 1e-9:
    return None
  n_pts = int(np.sum(k[m] > 0.7 * kmax))
  if n_pts < VISION_TURN_MIN_PTS:
    return None
  s_entry = float(s[np.nonzero(m)[0][0]])
  s_peak = float(s[int(np.argmax(k))])
  return kmax, s_peak, s_entry, n_pts


def vision_turn_request(xs, ys, v_ego, a_dec_max=None, a_lat=None, min_radius=None,
                        lookahead_max=None, min_d=None):
  """Deceleration this frame's model path asks for, or None if there is no bend worth it.

  Returns a dict so the caller can log why it acted:
    a_req        <= 0 m/s^2 the planner should treat as an upper bound
    v_cap        comfort speed in the bend, m/s
    kappa        curvature of the bend, 1/m
    d            metres to the bend entry
    s_peak       metres to the tightest point
    feasibility  'early' (can reach v_cap before the bend) or 'reactive' (too close already,
                 so the policy asks its comfort decel and no more)
  """
  a_dec_max = VISION_TURN_A_DEC_MAX if a_dec_max is None else a_dec_max
  a_lat = VISION_TURN_A_LAT if a_lat is None else a_lat
  min_radius = VISION_TURN_MIN_RADIUS if min_radius is None else min_radius
  lookahead_max = VISION_TURN_LOOKAHEAD_MAX if lookahead_max is None else lookahead_max
  min_d = VISION_TURN_MIN_D if min_d is None else min_d

  if not VISION_TURN_SPEED_ENABLED:
    return None
  geom = path_curvature(xs, ys, lookahead_max)
  if geom is None:
    return None
  s, _y, kappa = geom
  bend = find_bend(s, kappa, min_radius)
  if bend is None:
    return None
  kmax, s_peak, s_entry, _n = bend

  v_cap = math.sqrt(a_lat / kmax)
  d = max(min_d, s_entry)
  if v_ego <= v_cap:
    return {"a_req": 0.0, "v_cap": v_cap, "kappa": kmax, "d": d, "s_peak": s_peak,
            "feasibility": "inside"}
  a_needed = (v_cap * v_cap - v_ego * v_ego) / (2.0 * d)
  feasible = a_needed >= -a_dec_max
  return {"a_req": max(a_needed, -a_dec_max), "v_cap": v_cap, "kappa": kmax, "d": d,
          "s_peak": s_peak, "feasibility": "early" if feasible else "reactive"}


class VisionTurnSpeed:
  """One-frame wrapper holding the only state the policy needs: a short hold timer.

  `update()` returns the deceleration request (<= 0) the planner should take as an upper
  bound, or None when stock behaviour applies. The hold timer exists because the model's
  path is noisy frame to frame; without it the request can flicker off for a single frame in
  the middle of a slowdown.
  """

  def __init__(self, hold_s=None):
    self.hold_s = hold_s
    self.active = False
    self.hold_until = 0.0
    self.last = None

  def reset(self):
    self.active = False
    self.hold_until = 0.0
    self.last = None

  def update(self, model_x, model_y, v_ego, now_t=0.0):
    if not VISION_TURN_SPEED_ENABLED:
      self.reset()
      return None
    hold_s = VISION_TURN_HOLD_S if self.hold_s is None else self.hold_s
    req = vision_turn_request(model_x, model_y, v_ego)
    if req is not None and req["a_req"] < 0.0:
      self.active = True
      self.hold_until = now_t + hold_s
      self.last = req
      return req
    if self.active and now_t < self.hold_until and self.last is not None:
      # hold the previous request: no bend detected this frame, but one was a moment ago
      held = dict(self.last)
      held["held"] = True
      return held
    self.reset()
    return None
