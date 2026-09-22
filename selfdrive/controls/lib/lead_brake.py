#!/usr/bin/env python3
"""Brake earlier for a lead that is slowing - a queue at a light, or anything else.

This fork already carries a late, hard "danger override" in `longitudinal_planner.py`: it waits
until the lead is closing at 4.5 m/s (16 km/h) and then ramps a -1.0 to -1.2 m/s^2 request. That
threshold is the problem for the case the owner actually complains about: a car slowing to a
stop in front closes at 1-2 m/s, so the trigger never arms and the usable response is a late one.

This module is the same idea, moved earlier. It projects the lead's own deceleration forward
(`LOOK_S`) and asks for the deceleration needed to match that projected speed by the time the car
reaches a minimum gap behind it:

    v_lead_pred = max(0, v_lead + a_lead * LOOK_S)
    a_req       = (v_lead_pred^2 - v_ego^2) / (2 * max(dRel - MIN_GAP, floor))

so the request is silent in steady following (v_ego ~ v_lead), grows as soon as the lead starts
braking - earlier and gentler than waiting for a closing-speed threshold - and self-terminates as
the car slows to the projected speed. It is a cap, never a command: the planner takes it only as an
upper bound on its own output, and the car's set speed is untouched.

Shipped OFF (LEAD_BRAKE_ENABLED = 0 = stock behaviour). Knobs live in the same JSON the lateral
and turn-speed tuning read, every range one-sided.
"""
import json
import math

import numpy as np

# ---------------------------------------------------------------------------
# tuning; every value has an inert state, and every range below is one-sided
# ---------------------------------------------------------------------------
LEAD_BRAKE_ENABLED = 0            # 0 = off (stock), 1 = on
LEAD_BRAKE_LOOK_S = 1.0           # s; how far ahead the lead's own deceleration is projected
LEAD_BRAKE_MIN_GAP_M = 6.0        # m; the gap the formula aims to match the lead's speed by
LEAD_BRAKE_TRIGGER = 0.25         # m/s^2; quieter than this and the policy says nothing
LEAD_BRAKE_A_DEC_MAX = 1.5        # m/s^2; the most this policy may ask for on its own
LEAD_BRAKE_HOLD_S = 0.5           # s; keep an in-progress request through one noisy frame
LEAD_BRAKE_MIN_V = 2.0            # m/s; below this the plan's own low-speed logic belongs

TUNING_PATH = "/data/hermes/tuning.json"
TUNING_LIMITS = {
  "LEAD_BRAKE_ENABLED": (0.0, 1.0),        # 0 = off (stock)
  "LEAD_BRAKE_LOOK_S": (0.0, 1.0),         # less projection = a later, gentler response
  "LEAD_BRAKE_MIN_GAP_M": (6.0, 20.0),     # aim to match speed further back
  "LEAD_BRAKE_TRIGGER": (0.25, 1.0),       # a higher bar means it acts less often
  "LEAD_BRAKE_A_DEC_MAX": (0.5, 1.5),      # the shipped 1.5 m/s^2 is the ceiling
  "LEAD_BRAKE_HOLD_S": (0.0, 0.5),         # less memory; 0.0 = stateless
}

TUNING_BASE = {
  "LEAD_BRAKE_ENABLED": LEAD_BRAKE_ENABLED,
  "LEAD_BRAKE_LOOK_S": LEAD_BRAKE_LOOK_S,
  "LEAD_BRAKE_MIN_GAP_M": LEAD_BRAKE_MIN_GAP_M,
  "LEAD_BRAKE_TRIGGER": LEAD_BRAKE_TRIGGER,
  "LEAD_BRAKE_A_DEC_MAX": LEAD_BRAKE_A_DEC_MAX,
  "LEAD_BRAKE_HOLD_S": LEAD_BRAKE_HOLD_S,
}


def read_tuning(path=TUNING_PATH, limits=TUNING_LIMITS):
  """{name: clamped value} for the keys the tuning file carries, else {}. Never raises."""
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


def apply_tuning(tuning):
  """Push tuning values into the module globals this policy reads each call."""
  globals()["LEAD_BRAKE_ENABLED"] = int(tuning.get("LEAD_BRAKE_ENABLED", TUNING_BASE["LEAD_BRAKE_ENABLED"]) >= 0.5)
  globals()["LEAD_BRAKE_LOOK_S"] = tuning.get("LEAD_BRAKE_LOOK_S", TUNING_BASE["LEAD_BRAKE_LOOK_S"])
  globals()["LEAD_BRAKE_MIN_GAP_M"] = tuning.get("LEAD_BRAKE_MIN_GAP_M", TUNING_BASE["LEAD_BRAKE_MIN_GAP_M"])
  globals()["LEAD_BRAKE_TRIGGER"] = tuning.get("LEAD_BRAKE_TRIGGER", TUNING_BASE["LEAD_BRAKE_TRIGGER"])
  globals()["LEAD_BRAKE_A_DEC_MAX"] = tuning.get("LEAD_BRAKE_A_DEC_MAX", TUNING_BASE["LEAD_BRAKE_A_DEC_MAX"])
  globals()["LEAD_BRAKE_HOLD_S"] = tuning.get("LEAD_BRAKE_HOLD_S", TUNING_BASE["LEAD_BRAKE_HOLD_S"])


def lead_brake_request(v_ego, lead, look_s=None, min_gap=None, trigger=None, a_dec_max=None,
                       min_v=None):
  """Deceleration this frame asks for because the lead is slowing, or None.

  `lead` is something with status/dRel/vRel/vLead/aLeadK (the fork's `radarState.leadOne`).
  Returns a dict so the caller can log why it acted: a_req, gap, v_lead_pred, ttc,
  feasibility ('early' if the request fits inside the comfort cap, else 'reactive').
  """
  look_s = LEAD_BRAKE_LOOK_S if look_s is None else look_s
  min_gap = LEAD_BRAKE_MIN_GAP_M if min_gap is None else min_gap
  trigger = LEAD_BRAKE_TRIGGER if trigger is None else trigger
  a_dec_max = LEAD_BRAKE_A_DEC_MAX if a_dec_max is None else a_dec_max
  min_v = LEAD_BRAKE_MIN_V if min_v is None else min_v

  if not LEAD_BRAKE_ENABLED:
    return None
  if lead is None or not getattr(lead, "status", False):
    return None
  if v_ego < min_v:
    return None

  d_rel = float(getattr(lead, "dRel", 0.0))
  v_lead = float(getattr(lead, "vLead", 0.0))
  a_lead = float(getattr(lead, "aLeadK", 0.0))
  v_rel = float(getattr(lead, "vRel", 0.0))
  v_lead_pred = max(0.0, v_lead + a_lead * look_s)

  if v_ego <= v_lead_pred:
    # not closing on where the lead will be: nothing to ask for
    return None

  d_eff = max(d_rel - min_gap, 1.0)
  a_req = (v_lead_pred * v_lead_pred - v_ego * v_ego) / (2.0 * d_eff)
  if a_req >= -trigger:
    return None
  ttc = d_rel / -v_rel if v_rel < -0.05 else None
  return {"a_req": max(a_req, -a_dec_max), "gap": d_rel, "v_lead_pred": v_lead_pred,
          "ttc": ttc, "feasibility": "early" if a_req >= -a_dec_max else "reactive"}


class LeadBrake:
  """One-frame wrapper holding a hold timer, so one noisy frame cannot cancel a slowdown."""

  def __init__(self, hold_s=None):
    self.hold_s = hold_s
    self.active = False
    self.hold_until = 0.0
    self.last = None

  def reset(self):
    self.active = False
    self.hold_until = 0.0
    self.last = None

  def update(self, v_ego, lead, now_t=0.0):
    if not LEAD_BRAKE_ENABLED:
      self.reset()
      return None
    hold_s = LEAD_BRAKE_HOLD_S if self.hold_s is None else self.hold_s
    req = lead_brake_request(v_ego, lead)
    if req is not None:
      self.active = True
      self.hold_until = now_t + hold_s
      self.last = req
      return req
    if self.active and now_t < self.hold_until and self.last is not None:
      held = dict(self.last)
      held["held"] = True
      return held
    self.reset()
    return None
