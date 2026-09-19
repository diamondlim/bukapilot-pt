#!/usr/bin/env python3
import json
import math
import time
from numbers import Number

import numpy as np

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.conditional_experimental_mode import ConditionalExperimentalMode
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


# ── Lane-centre correction ──────────────────────────────────────────────────────
# Reads the model's LANE LINES — lane geometry measured from the car — rather than the
# model's own planned path. The plan is already lane-centred, so a correction derived from
# it just re-adds the plan's own curvature (a ~15% curvature amplifier, not a centring
# loop). Offset is sampled at a lookahead distance and converted with pure pursuit:
#   curvature = 2 * offset / lookahead^2
# Frame conventions, measured on this car's own logs (not assumed):
#   laneLines[1] is the boundary on the car's LEFT and laneLines[2] the one on its RIGHT —
#   the pair modeld itself uses for lane_line_meta.leftY/rightY — and +y points to the RIGHT.
#   A positive curvature bends the path toward +y, i.e. to the right.
# So a positive centre offset (the lane centre sits to the car's right) is corrected with a
# positive curvature, and this gain stays positive.
#
# The gain matters: with the correct pair the offset is small (median 0.18 m on logged
# drives), and pure pursuit over a 30 m lookahead turns that into roughly 0.024 m/s^2 of
# extra lateral acceleration at 0.15 — i.e. nothing. 0.5 gives ~0.08 m/s^2 for an 18 cm
# offset, still well inside the 0.3 m/s^2 budget; raise toward 1.0 for a firmer centring.
# A compile-time constant, deliberately. A runtime param would need a new key in
# common/params_keys.h, and on a prebuilt image that changes the compiled key list, so the tree
# stops matching the build state the manager validates at startup. Keeping this branch
# python-only means an install needs no rebuild. Changing the gain costs one commit, not a write.
#
# 1.0 = the correction spends its whole budget on the offsets the car actually drives with.
# Chosen from 92,996 engaged frames of the owner's own logs: mean |lane-centre offset| 0.27 m,
# p90 0.74 m, and 21% of the time beyond 0.4 m. At 0.25 the correction produced 0.035 m/s^2
# against the ~0.24 m/s^2 a 0.27 m offset needs, i.e. about a seventh of the requirement - which
# is why the earlier tuned value was never felt. 0.0 remains the off switch (stock behaviour).
LANE_CORRECTION_GAIN = 1.0
LANE_CORRECTION_LOOKAHEAD_S = 1.5   # horizon (at current speed) used for the conversion
LANE_CORRECTION_MIN_PROB = 0.5      # both lane lines must be at least this probable
LANE_CORRECTION_MAX_OFFSET_M = 1.5  # reject implausible lane-centre offsets
LANE_CORRECTION_MAX_LAT_ACC = 0.3   # m/s^2 of extra lateral acceleration the correction may ask for
LANE_CORRECTION_MIN_SPEED = 5.0     # m/s
LANE_CORRECTION_MIN_LOOKAHEAD_M = 10.0  # floor on the lookahead at low speed

# Memory and shaping for the lane geometry. Lane lines are noisy frame to frame — worse at
# night — so the offset is low-passed rather than used raw. The first attempt used a sliding
# median: good noise rejection, but its output is piecewise-constant — every window slide
# moves the value in a step, and a step in the injected curvature is felt as steering jerk.
# A first-order filter has no steps, and the slew limiter caps the correction's own jerk by
# construction. Both are switchable: tau 0.0 acts on each frame raw, rate 0.0 is unlimited.
# On a brief dropout the filtered value is held for at most LANE_MEMORY_HOLD_S, and only
# while going straight: a stale offset carried into a corner would steer toward where the
# lane used to be. A lane change or a longer dropout clears the memory.
LANE_CORRECTION_FILTER_TAU_S = 0.5   # s; first-order time constant (0.0 -> use each frame raw)
LANE_CORRECTION_MAX_ACC_RATE = 0.9   # m/s^2 per s; how fast the correction may change (0.0 -> unlimited)
LANE_MEMORY_HOLD_S = 0.4             # how long a remembered offset survives a dropout
LANE_MEMORY_MAX_YAW_RATE = 0.15      # rad/s; above this, never trust a stale offset

# Runtime tuning of everything above. The values are re-read about once a second from a small JSON
# file, so a knob can be moved while driving and A/B'd without a redeploy or a branch per value.
# Semantics that keep this safe on a car:
#   * every key defaults to the constant above, and any key missing from the file (or a file that
#     is absent, malformed, or out of range) leaves the shipped value in place - the off switch is
#     "delete the file", not "set a flag";
#   * values are clamped to a per-key range, so no tuning can ask for something implausible;
#   * nothing here can raise: a bad tuning file must never be able to take controlsd down.
# Written by the Bluetooth settings service (which reports each key as live only once this file is
# actually being read), so the phone is never offered a knob the car ignores.
TUNING_PATH = "/data/hermes/tuning.json"
TUNING_RELOAD_FRAMES = 100          # ~1 s at DT_CTRL = 0.01
# Every range is one-sided on purpose: the file may move the correction toward the gentler side
# (less authority, more smoothing, stricter geometry tests) or switch it off entirely, but it can
# never make it sharper than the code committed here. Turning something *up* is a change to the
# car's lateral behaviour, and that belongs in a commit with a drive behind it - not in a slider.
TUNING_LIMITS = {
  "LANE_CORRECTION_GAIN": (0.0, 1.0),                 # 0.0 = off (stock)
  "LANE_CORRECTION_LOOKAHEAD_S": (1.5, 3.0),          # further ahead = gentler
  "LANE_CORRECTION_MIN_PROB": (0.5, 0.99),            # stricter lane-line confidence
  "LANE_CORRECTION_MAX_OFFSET_M": (0.2, 1.5),         # stricter plausibility bound
  "LANE_CORRECTION_MAX_LAT_ACC": (0.0, 0.3),          # the shipped budget is the ceiling
  "LANE_CORRECTION_MIN_SPEED": (5.0, 20.0),           # raise the speed the correction acts at
  "LANE_CORRECTION_FILTER_TAU_S": (0.5, 2.0),         # more smoothing
  "LANE_CORRECTION_MAX_ACC_RATE": (0.0, 0.9),         # slower changes; 0.0 = unlimited
  "LANE_MEMORY_HOLD_S": (0.0, 0.4),                   # hold a stale offset for less time
  "LANE_MEMORY_MAX_YAW_RATE": (0.0, 0.15),            # trust the held offset in less curvature
}
TUNING_BASE = {name: globals()[name] for name in TUNING_LIMITS}


def read_tuning(path=TUNING_PATH, limits=TUNING_LIMITS):
  """{name: clamped value} for the keys the tuning file actually carries, else {}.

  Pure and importable on its own so the validation is unit-testable off the car.
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


def lane_centre_offset(model_v2, v_ego):
  """Lateral offset of the lane centre from the car, from the model's current-lane lines.

  Uses only the pair that bounds the car's own lane: laneLines[1] (left) and laneLines[2]
  (right). Bracketing the car with the max/min of every probable line — the previous
  behaviour — is wrong: 79% of logged frames carry more than two probable lines, and the
  extremes then reach across the adjacent lanes. Measured on this car's own drives that
  inflated the offset from a true median of 0.18 m to 0.55-1.26 m and biased it to the
  right, which in a tight lane (where extra lines are most visible) held the car right of
  centre. Returns None rather than guessing when the pair is not trustworthy.
  """
  probs = list(model_v2.laneLineProbs)
  lines = list(model_v2.laneLines)
  if len(probs) < 3 or len(lines) < 3:
    return None

  lookahead = max(float(v_ego) * LANE_CORRECTION_LOOKAHEAD_S, LANE_CORRECTION_MIN_LOOKAHEAD_M)

  def y_at_lookahead(line):
    xs = np.asarray(line.x, dtype=float)
    ys = np.asarray(line.y, dtype=float)
    if xs.size < 2 or ys.size != xs.size:
      return None
    order = np.argsort(xs)
    xs, ys = xs[order], ys[order]
    if not (xs[0] <= lookahead <= xs[-1]):
      return None
    y = float(np.interp(lookahead, xs, ys))
    return y if math.isfinite(y) else None

  y_left = y_at_lookahead(lines[1]) if probs[1] >= LANE_CORRECTION_MIN_PROB else None
  y_right = y_at_lookahead(lines[2]) if probs[2] >= LANE_CORRECTION_MIN_PROB else None
  if y_left is None or y_right is None:
    return None                                  # never mix in adjacent-lane lines
  if y_left >= y_right:
    return None                                  # left line must sit at the lower y (+y is right)
  if not 1.5 <= (y_right - y_left) <= 6.0:
    return None                                  # implausible lane width

  centre_offset = 0.5 * (y_left + y_right)       # + means the lane centre is right of the car
  if abs(centre_offset) > LANE_CORRECTION_MAX_OFFSET_M:
    return None

  return centre_offset


def lane_curvature_from_offset(centre_offset, v_ego, lookahead):
  """Pure-pursuit curvature for an offset, bounded by a lateral-acceleration budget.

  Bounding by acceleration rather than by a flat curvature cap keeps the correction gentle
  at speed instead of clamping it everywhere.
  """
  curvature = 2.0 * centre_offset / (lookahead ** 2)
  max_curvature = LANE_CORRECTION_MAX_LAT_ACC / max(float(v_ego) ** 2, 1.0)
  return float(np.clip(curvature, -max_curvature, max_curvature))


def lane_extra_lat_acc(centre_offset, v_ego, lookahead):
  """The extra lateral acceleration the correction asks for, m/s^2.

  Same magnitude as before (the clamp lives in lane_curvature_from_offset), but expressed as
  acceleration so a slew limit can bound it in physical units rather than in curvature.
  """
  curvature = lane_curvature_from_offset(centre_offset, v_ego, lookahead)
  return float(curvature * max(float(v_ego) ** 2, 1.0))


def lane_centre_curvature(model_v2, v_ego):
  """One-shot lane-centre curvature, for callers that keep no memory."""
  centre_offset = lane_centre_offset(model_v2, v_ego)
  if centre_offset is None:
    return None
  lookahead = max(float(v_ego) * LANE_CORRECTION_LOOKAHEAD_S, LANE_CORRECTION_MIN_LOOKAHEAD_M)
  return lane_curvature_from_offset(centre_offset, v_ego, lookahead)


class LaneCentreMemory:
  """Lane-centre offset with memory: a first-order filter plus a bounded hold.

  Offset sign follows lane_centre_offset (+ = lane centre is left of the car).
  """

  def __init__(self, tau_s=LANE_CORRECTION_FILTER_TAU_S, hold_s=LANE_MEMORY_HOLD_S, dt=DT_CTRL):
    self.alpha = 1.0 - math.exp(-dt / tau_s) if tau_s > 0.0 else 1.0
    self.hold_s = hold_s
    self._value = None
    self._samples = 0
    self._last_good_t = None

  def reset(self):
    """Forget everything — used on disengage, low speed and lane changes."""
    self._value = None
    self._samples = 0
    self._last_good_t = None

  @property
  def samples(self):
    return self._samples

  def update(self, model_v2, v_ego, yaw_rate, now):
    """Filtered lane-centre offset for this frame, or None when there is nothing to use.

    `now` is a monotonic clock in seconds; injecting it keeps the hold testable.
    """
    offset = lane_centre_offset(model_v2, v_ego)
    if offset is not None:
      self._value = offset if self._value is None else self._value + self.alpha * (offset - self._value)
      self._samples += 1
      self._last_good_t = now
      return self._value

    # Dropout: reuse the filtered value only briefly, and only while going straight.
    if (self._last_good_t is not None and (now - self._last_good_t) <= self.hold_s
        and abs(yaw_rate) <= LANE_MEMORY_MAX_YAW_RATE and self._value is not None):
      return self._value

    self.reset()
    return None


class LaneAccelSlew:
  """Bounds how fast the correction's own lateral-acceleration request may change.

  The correction asks for a small extra lateral acceleration; the *rate* at which that
  request moves is what a torque controller turns into steering jerk. Capping the rate makes
  the correction's jerk bounded by construction, whatever the lane geometry does.
  """

  def __init__(self, max_rate=LANE_CORRECTION_MAX_ACC_RATE, dt=DT_CTRL):
    self.max_rate = max_rate
    self.dt = dt
    self._value = 0.0

  def reset(self):
    self._value = 0.0

  @property
  def value(self):
    return self._value

  def limit(self, requested):
    """Return the rate-limited request, in m/s^2."""
    if self.max_rate > 0.0:
      step = self.max_rate * self.dt
      self._value += float(np.clip(float(requested) - self._value, -step, step))
    else:
      self._value = float(requested)
    return self._value


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'radarState'], poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'])

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None
    self.cem = ConditionalExperimentalMode()
    self.lane_centre = LaneCentreMemory()
    self.lane_slew = LaneAccelSlew()
    self.lane_correction_gain = LANE_CORRECTION_GAIN
    self._tuning_frame = 0
    self.apply_lane_tuning(read_tuning())

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI, DT_CTRL)

  def apply_lane_tuning(self, tuning):
    """Push the tuned values to where the correction actually reads them.

    Two places, and both are needed: the module constants are looked up in globals() per call, but
    the gain, the filter's alpha and the slew limiter's rate were captured when those objects were
    constructed, so a change to the constant alone would not be felt. Every tunable is reset to its
    shipped constant first, so removing a key from the file restores the default rather than
    freezing the last tuned value.
    """
    for name, base in TUNING_BASE.items():
      globals()[name] = tuning.get(name, base)
    self.lane_correction_gain = tuning.get("LANE_CORRECTION_GAIN", TUNING_BASE["LANE_CORRECTION_GAIN"])
    tau = tuning.get("LANE_CORRECTION_FILTER_TAU_S", TUNING_BASE["LANE_CORRECTION_FILTER_TAU_S"])
    self.lane_centre.alpha = 1.0 - math.exp(-DT_CTRL / tau) if tau > 0.0 else 1.0
    self.lane_centre.hold_s = tuning.get("LANE_MEMORY_HOLD_S", TUNING_BASE["LANE_MEMORY_HOLD_S"])
    self.lane_slew.max_rate = tuning.get("LANE_CORRECTION_MAX_ACC_RATE",
                                         TUNING_BASE["LANE_CORRECTION_MAX_ACC_RATE"])

  def update(self):
    self._tuning_frame += 1
    if self._tuning_frame >= TUNING_RELOAD_FRAMES:
      self._tuning_frame = 0
      self.apply_lane_tuning(read_tuning())
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    self.cem.update(self.sm['carState'], self.sm['radarState'].leadOne, self.sm['modelV2'], self.sm['selfdriveState'])

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    CC.latActive = self.sm['selfdriveState'].active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.CP.steerAtStandstill)
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and self.CP.openpilotLongitudinalControl

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    freeze_long_i = bool(self.sm['carOutput'].stockLongitudinalContributing)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop,
                                            pid_accel_limits, freeze_integrator=freeze_long_i))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage
    new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature

    # Lane-centre correction: nudge the car back toward the middle of the lane using the
    # model's current-lane line pair, through a one-second memory so it tracks lane geometry
    # instead of frame-to-frame noise. Skipped during lane changes (which also clears the
    # memory) and at low speed; the estimator returns None when the geometry cannot be
    # trusted. Sign: +y is to the car's right and a positive curvature bends right, so a
    # positive offset is corrected with a positive curvature.
    # The knobs are re-read about once a second (apply_lane_tuning below): a write to the tuning
    # file takes effect without a redeploy, and with the gain at 0 the block is skipped entirely.
    # With the gain at 0 the block below is skipped entirely and the memory is held reset, so
    # desired curvature is the model's own — identical to the fork's stock lateral behaviour.
    if self.lane_correction_gain > 0.0 and CC.latActive and CS.vEgo > LANE_CORRECTION_MIN_SPEED and model_v2.meta.laneChangeState == LaneChangeState.off:
      centre_offset = self.lane_centre.update(model_v2, CS.vEgo, float(model_v2.orientationRate.z[0]),
                                              time.monotonic())
      if centre_offset is not None:
        lookahead = max(CS.vEgo * LANE_CORRECTION_LOOKAHEAD_S, LANE_CORRECTION_MIN_LOOKAHEAD_M)
        extra_acc = self.lane_correction_gain * lane_extra_lat_acc(centre_offset, CS.vEgo, lookahead)
        extra_acc = self.lane_slew.limit(extra_acc)
        new_desired_curvature += extra_acc / max(float(CS.vEgo) ** 2, 1.0)
    else:
      self.lane_centre.reset()
      self.lane_slew.reset()

    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       curvature_limited, lat_delay)
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
