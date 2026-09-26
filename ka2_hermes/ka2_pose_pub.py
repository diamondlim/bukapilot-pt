#!/usr/bin/env python3
"""Publish the driving model's lane geometry for the phone app (one line of JSON, ~10 Hz).

The phone gets the lane view without video: the model already knows the lane lines, the lane
width and where the car sits in them, so a few hundred bytes a second over the Bluetooth serial
link is enough to draw the lane the car is actually steering from. Trivial for SPP; hopeless for
video, which is why that half lives over Wi-Fi instead.

Interpreter: this must run under the box's fork venv (`/usr/local/venv/bin/python3`) because
`cereal` needs `capnp`, which the system python (`/usr/bin/python3`, the one with dbus/gi that
the Bluetooth service runs as) does not have. Two interpreters, so the wire between them is a
file: this publisher writes /dev/shm/ka2_pose.json atomically and the Bluetooth service forwards
the newest line to whichever client asked for it. Offroad there is no modelV2 at all, so the file
says ok=0 with a reason rather than pretending - the vendor app's "MAX --" on the bench is the
same fact.
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from acc_decode import ACC_CMD_ADDRESS, decode_acc_cmd, frame_is_sane

POSE_PATH = os.environ.get("KA2_POSE_PATH", "/dev/shm/ka2_pose.json")
RATE_HZ = 10.0
MAX_X = 50.0          # metres of road to send; the phone draws this window
MAX_POINTS = 20       # per lane line, so the JSON line stays small
NEAR_DENSE_M = 30.0   # keep every point this far out; thin beyond, where a few metres is invisible
OUTER_POINTS = 8      # the neighbouring lanes' outer edges: context, so sent sparsely
MIN_PROB = 0.5        # same threshold the car's own lane-centre correction uses
# The route is a TEN SECOND horizon, not a distance: measured live, 33 points spanning 10.0 s. Stopped
# that is 17 m, at 110 km/h it is 306 m. So this is a ceiling for the payload, not a judgement about
# what is worth showing - the model's own reach is the limit, and the app draws wherever it ends.
PATH_RANGE_M = 400.0   # how far ahead the model's own trajectory is worth drawing
PATH_POINTS = 20      # and how many points of it travel over Bluetooth
# The car's dashboard shows its own speed, and the fork says what that is: carstate sets
# vEgoCluster = vEgo * HUD_MULTIPLIER (opendbc/car/byd/values.py, 1.12 on these cars). vEgo is the true
# speed - wheel speed calibrated against GPS by wheelSpeedFactor - so the two differ by this factor on
# purpose, and the app shows both rather than pretending they are the same number.
DASH_MULTIPLIER = 1.12
SHOW_PROB = 0.05      # below this the model is guessing; between the two it is predicting, and that
                      # is worth showing on faint markings as long as it is never passed off as read
LOOKAHEAD_S = 1.5     # matches LANE_CORRECTION_LOOKAHEAD_S in controlsd.py
MIN_LOOKAHEAD_M = 10.0
MAX_OFFSET_M = 1.5
# Display-only fallbacks, used when the lane is not readable at the horizon the car itself uses.
# The model's lane lines get short at low speed (the car's own correction is inert then too: at a
# standstill only 1% of logged frames have a readable 10 m horizon, at 5-12 m/s 46%, above 12 m/s
# 96%), so the panel would otherwise sit empty exactly when a driver looks at it in town. Shorter
# horizons are tried in order and the answer is labelled with the distance it was measured at -
# never silently swapped for the car's own geometry, and never extrapolated past the lane's end.
FALLBACK_LOOKAHEADS = (8.0, 6.0, 4.0, 3.0, 2.0)


def _pairs(line):
  """(x, y) points of a model lane line, sorted by distance ahead. [] if unusable."""
  try:
    xs = [float(v) for v in line.x]
    ys = [float(v) for v in line.y]
  except Exception:
    return []
  if len(xs) < 2 or len(xs) != len(ys):
    return []
  pts = sorted(zip(xs, ys))
  return [(x, y) for x, y in pts if math.isfinite(x) and math.isfinite(y) and 0.0 <= x <= MAX_X]


def _thin(pts, limit=MAX_POINTS):
  """Keep the near field dense and the far field sparse.

  The app now draws the lane out to 100 m, but the near 30 m is what the car actually steers on, so
  uniform thinning to a small count (which is what this used to do, 10 points over 192 m, one every
  21 m) left the near lane as a straight line between two far-apart points.
  """
  if len(pts) <= limit:
    return pts
  near = [p for p in pts if p[0] <= NEAR_DENSE_M]
  far = [p for p in pts if p[0] > NEAR_DENSE_M]
  budget = max(limit - len(near), 2)
  if len(far) <= budget:
    return near + far
  step = (len(far) - 1) / float(budget - 1)
  return near + [far[min(len(far) - 1, int(round(i * step)))] for i in range(budget)]


def _y_at(pts, x):
  """Linear interpolation of y at x, or None when x is outside the line's own span."""
  if len(pts) < 2 or not (pts[0][0] <= x <= pts[-1][0]):
    return None
  for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
    if x0 <= x <= x1:
      if x1 == x0:
        return y0
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
  return None


def _measure(left_pts, right_pts, horizon, left_prob=1.0, right_prob=1.0):
  """(offset, width, None) for a usable lane centre at this distance, else (None, None, reason).

  The same tests the car's own helper applies - both lines at least MIN_PROB, the left line below
  the right one, a plausible width, an offset inside MAX_OFFSET_M - so a pose this accepts is one
  the car would also accept at that horizon. The reason is carried out rather than collapsed to a
  bool: a client showing "lane pair inverted" is useful, "no lane data" is not.
  """
  if not left_pts or not right_pts:
    return None, None, "not readable"
  # The confidence test has to be explicit. It used to be implied by the caller dropping the points,
  # so when a faint lane's points started being sent for display, a lane the car would never steer on
  # came back as "ok" - which would have told the driver the lane had been read when it had not.
  if min(left_prob, right_prob) < MIN_PROB:
    return None, None, "faint markings (%.2f / %.2f)" % (left_prob, right_prob)
  y_left = _y_at(left_pts, horizon)
  y_right = _y_at(right_pts, horizon)
  if y_left is None or y_right is None:
    return None, None, "not readable"
  if y_left >= y_right:
    return None, None, "lane pair inverted at %.0f m" % horizon
  width = y_right - y_left
  if not 1.5 <= width <= 6.0:
    return None, None, "implausible lane width %.2f m at %.0f m" % (width, horizon)
  offset = 0.5 * (y_left + y_right)
  if abs(offset) > MAX_OFFSET_M:
    return None, None, "implausible centre offset %.2f m at %.0f m" % (offset, horizon)
  return offset, width, None


def _centre_line(left_pts, right_pts, max_points=PATH_POINTS):
  """The lane centre, point by point: halfway between the pair the car's own correction uses.

  This is the "calculated lane" as opposed to the lane lines themselves - the same midpoint the fork's
  lane-centre helper works from (the publisher's pose-vs-logs test holds it to within 5 mm of the car's
  own numbers). The car additionally filters and rate-limits it before steering, which is not shown here.
  """
  out = []
  for x, y_left in left_pts:
    y_right = _y_at(right_pts, x)
    if y_right is None:
      continue
    out.append((x, 0.5 * (y_left + y_right)))
  if len(out) < 2:
    return []
  return _thin(out, max_points)


class CarStateCache:
  """The car's own values, held between frames.

  The publisher runs at 10 Hz while carState arrives on its own schedule, so reading "this cycle's
  message or nothing" made the speed and the set speed drop to zero on every cycle without a message -
  which the app showed as a readout that was wrong and took seconds to settle. A value once seen stays
  until it is replaced (and a lead is dropped only once it has gone stale).
  """

  LEAD_STALE_S = 1.5
  ACC_STALE_S = 1.0             # ACC_CMD runs at 50 Hz, so anything older than a second is gone

  def __init__(self):
    self.v_ego = 0.0
    self.set_speed = 0.0          # the car's own displayed set speed, m/s
    self.set_speed_true = 0.0     # the same, normalised by the fork's HUD multiplier
    self.engaged = False
    self.leads = []
    self.leads_at = 0.0
    self.acc = None               # the car's own ACC_CMD, newest sane frame
    self.acc_at = 0.0

  def update(self, sm, now):
    # .get, not []: a service that was not subscribed must not raise here
    if sm.updated.get("carState"):
      car = sm["carState"]
      self.v_ego = float(car.vEgo)
      # cruiseState.speedCluster is what the car's dashboard shows; cruiseState.speed is the fork's
      # normalisation of it (it divides by HUD_MULTIPLIER, 1.12 on this platform). The dashboard number
      # is the one the driver can check, so that is what the app is given as the set speed.
      self.set_speed = float(car.cruiseState.speedCluster)
      self.set_speed_true = float(car.cruiseState.speed)
    if sm.updated.get("selfdriveState"):
      self.engaged = bool(sm["selfdriveState"].active)
    if sm.updated.get("radarState"):
      leads = []
      for lead in (sm["radarState"].leadOne, sm["radarState"].leadTwo):
        if lead.status and float(lead.dRel) > 0:
          leads.append({"d": float(lead.dRel), "y": float(lead.yRel),
                        "vr": float(lead.vRel), "p": float(lead.modelProb)})
      self.leads = leads
      self.leads_at = now
    if self.leads and now - self.leads_at > self.LEAD_STALE_S:
      self.leads = []            # a lead that stopped being reported is gone, not frozen on screen
    if sm.updated.get("can"):
      payload = sm["can"]
      for frame in getattr(payload, "can", payload):
        if int(frame.address) != ACC_CMD_ADDRESS:
          continue
        dat = bytes(frame.dat)
        if not frame_is_sane(dat):
          continue               # not the frame we think it is: the DBC's constants did not match
        self.acc = decode_acc_cmd(dat)
        self.acc_at = now
    if self.acc is not None and now - self.acc_at > self.ACC_STALE_S:
      self.acc = None            # the car stopped saying: show nothing rather than an old request
    return self


def build_pose(model_v2, v_ego, engaged, now, set_speed=0.0, leads=None, set_speed_true=0.0,
               acc=None):
  """One compact pose record. Never raises: a missing or untrustworthy model says why.

  +y is to the car's right (the fork's own lane_line_meta names laneLines[1] left and
  laneLines[2] right), so the offset and the drawn lane use the same sign convention as the
  car's controller - the app must not re-invent it.
  """
  pose = {"t": round(now, 3), "ok": 0, "why": "", "v": 0.0, "eng": 0, "set": 0.0}
  # Reported before the model is consulted: the set speed is the car's business, not the model's, and
  # it is still worth showing when there is no lane to draw (parked, or the model has not started).
  if acc:                        # the car's own acceleration request, in the car's own units
    pose["acc"] = {"cmd": acc.get("cmd", 0), "on": acc.get("on", 0), "on2": acc.get("on2", 0),
                   "ctrl": acc.get("ctrl", 0), "ovr": acc.get("ovr", 0), "still": acc.get("still", 0)}
  try:
    pose["set"] = round(max(float(set_speed), 0.0), 2)
    pose["set_true"] = round(max(float(set_speed_true), 0.0), 2)
  except Exception:
    pose["set"] = 0.0
    pose["set_true"] = 0.0
  # Lead vehicles, from the car's own adaptive-cruise tracking (radarState). Sent before the model is
  # consulted because a lead is the car's business, not the model's - and on this platform the lead is
  # vision-sourced (radar is unavailable), so the client should label it as the car's tracking rather
  # than as radar.
  for index, lead in enumerate((leads or [])[:2]):
    try:
      distance = float(lead["d"])
      if distance <= 0:
        continue
      entry = {"d": round(distance, 1), "y": round(float(lead["y"]), 2)}
      if lead.get("vr") is not None:
        entry["vr"] = round(float(lead["vr"]), 2)
      if lead.get("p") is not None:
        entry["p"] = round(float(lead["p"]), 2)
      pose["lead" if index == 0 else "lead2"] = entry
    except Exception:
      continue

  if model_v2 is None:
    pose["why"] = "no model (car offroad)"
    return pose

  try:
    pose["v"] = round(float(v_ego), 2)
    pose["vd"] = round(float(v_ego) * DASH_MULTIPLIER, 2)   # what the car's own display reads
    pose["eng"] = 1 if engaged else 0
    probs = [float(p) for p in model_v2.laneLineProbs]
    lines = list(model_v2.laneLines)
    if len(probs) < 3 or len(lines) < 3:
      pose["why"] = "model has no lane-line pair"
      return pose

    left_ok = probs[1] >= MIN_PROB
    right_ok = probs[2] >= MIN_PROB
    # What the car will not steer on, the driver may still want to see. On faint markings the lane
    # lines fall below MIN_PROB and used to vanish entirely, leaving an empty road: the model is still
    # predicting a lane, so it is sent - marked with "low" so the phone draws it as a prediction and
    # never as something that was read. Below SHOW_PROB it is a guess and nothing is sent.
    predicted = not (left_ok and right_ok) and max(probs[1], probs[2]) >= SHOW_PROB
    left_pts = _pairs(lines[1]) if (left_ok or predicted) else []
    right_pts = _pairs(lines[2]) if (right_ok or predicted) else []
    pose["lp"] = round(probs[1], 2)
    pose["rp"] = round(probs[2], 2)
    if predicted:
      pose["low"] = 1
    if left_pts:
      pose["l"] = [[round(x, 1), round(y, 2)] for x, y in _thin(left_pts)]
    if right_pts:
      pose["r"] = [[round(x, 1), round(y, 2)] for x, y in _thin(right_pts)]

    # The model also gives the far edge of the lane on each side (lines 0 and 3), which is what lets
    # the view show three lanes rather than one. They are context, not control, so they are sent more
    # sparsely - the payload travels over Bluetooth at 10 Hz and the ego pair is what the car steers on.
    if len(probs) >= 4:
      if probs[0] >= MIN_PROB:
        outer_left = _pairs(lines[0])
        if outer_left:
          pose["l2"] = [[round(x, 1), round(y, 2)] for x, y in _thin(outer_left, OUTER_POINTS)]
          pose["lp2"] = round(probs[0], 2)
      if probs[3] >= MIN_PROB:
        outer_right = _pairs(lines[3])
        if outer_right:
          pose["r2"] = [[round(x, 1), round(y, 2)] for x, y in _thin(outer_right, OUTER_POINTS)]
          pose["rp2"] = round(probs[3], 2)

    # The model's own predicted trajectory: where it expects the car to be in the next seconds. A
    # curvature circle was standing in for this before; these are the model's actual points.
    try:
      px = [float(v) for v in model_v2.position.x]
      py = [float(v) for v in model_v2.position.y]
      path_pts = [(x, y) for x, y in zip(px, py) if 0.5 <= x <= PATH_RANGE_M]
      if len(path_pts) >= 2:
        pose["path"] = [[round(x, 1), round(y, 2)] for x, y in _thin(path_pts, PATH_POINTS)]
        pose["path_reach"] = round(path_pts[-1][0], 1)
    except Exception:
      pass

    # The calculated lane: the centreline between the pair the car's own correction uses, sent only
    # when that pair is one the car would itself trust.
    if left_ok and right_ok and left_pts and right_pts:
      centre = _centre_line(left_pts, right_pts)
      if centre:
        pose["calc"] = [[round(x, 1), round(y, 2)] for x, y in centre]

    car_lookahead = max(float(v_ego) * LOOKAHEAD_S, MIN_LOOKAHEAD_M)
    pose["car_look"] = round(car_lookahead, 1)

    measured, reasons = None, []
    for horizon in (car_lookahead,) + FALLBACK_LOOKAHEADS:
      if horizon > car_lookahead:
        continue
      offset, width, reason = _measure(left_pts, right_pts, horizon, probs[1], probs[2])
      if reason is None:
        measured = (horizon, offset, width)
        break
      reasons.append(reason)
    if measured is None:
      # Prefer a specific complaint (inverted pair, impossible width) over "not readable", which is
      # what every parked frame looks like.
      specific = [r for r in reasons if r != "not readable"]
      pose["why"] = specific[0] if specific else (
          "lane pair not readable at or below %.0f m" % car_lookahead)
      return pose
    lookahead, offset, width = measured
    pose["look"] = round(lookahead, 1)
    if lookahead < car_lookahead:
      pose["fallback"] = 1

    pose["off"] = round(offset, 3)
    pose["w"] = round(width, 2)
    try:
      pose["curv"] = round(float(model_v2.action.desiredCurvature), 6)
    except Exception:
      pass
    pose["ok"] = 1
  except Exception as exc:                      # never take the publisher down over one frame
    pose["why"] = "read failed: %s" % exc
    pose["ok"] = 0
  return pose


def write_atomic(path, payload):
  tmp = path + ".tmp"
  with open(tmp, "w") as fh:
    fh.write(payload)
  os.replace(tmp, path)


def publish_failure(reason):
  write_atomic(POSE_PATH, json.dumps({"t": round(time.time(), 3), "ok": 0, "why": reason,
                                      "v": 0.0, "eng": 0}))


def main():
  try:
    from cereal import messaging
  except Exception as exc:
    print("cereal unavailable: %s" % exc, flush=True)
    while True:
      publish_failure("publisher has no cereal: %s" % exc)
      time.sleep(5)

  # radarState carries the lead vehicles; on this car it is absent offroad, so it must stay optional
  ignored = ["modelV2", "carState", "selfdriveState", "radarState"]
  # `can` carries the car's own transmissions, including ACC_CMD - the message its ADAS module uses
  # to ask for acceleration or braking. Reading it is the whole point: it is what the car is doing on
  # its own, and it is the interface a longitudinal build would take over.
  sm = messaging.SubMaster(["modelV2", "carState", "selfdriveState", "radarState", "can"],
                           ignore_alive=ignored + ["can"])
  print("publishing %s at %.0f Hz" % (POSE_PATH, RATE_HZ), flush=True)
  cache = CarStateCache()
  period = 1.0 / RATE_HZ
  while True:
    started = time.monotonic()
    try:
      sm.update(60)
      now = time.time()
      car = cache.update(sm, now)
      model_v2 = sm["modelV2"] if sm.updated["modelV2"] else None
      write_atomic(POSE_PATH, json.dumps(
          build_pose(model_v2, car.v_ego, car.engaged, now, car.set_speed, car.leads,
                     car.set_speed_true, car.acc)))
    except Exception as exc:
      publish_failure("publisher error: %s" % exc)
    time.sleep(max(0.0, period - (time.monotonic() - started)))
  return 0


if __name__ == "__main__":
  sys.exit(main())
