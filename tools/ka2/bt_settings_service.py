#!/usr/bin/env python3
"""Bluetooth settings endpoint for the KA2 (SPP / RFCOMM over BlueZ 5.72).

BlueZ 5.7x refuses service records registered the old way (`sdptool add SP` silently does
nothing), so this registers org.bluez.Profile1 over D-Bus with the SPP UUID instead.

Line protocol on the serial link (one reply line per command, banner on connect):

    HELP                        -> this text
    INFO                        -> one `I {json}` line: the device facts the phone's
                                   "Connected Device" card shows
    SCHEMA                      -> one `K {json}` row per setting, then `E {"n":N}`. Each row
                                   declares its own type, range, section and - the point of the
                                   exercise - whether a write is *honest*: `live` (takes effect
                                   now), `inert` (accepted and recorded, but this build ignores
                                   it, so the phone greys it out) or `ro` (the box writes it
                                   itself, so it is not offered as a control at all)
    LIST                        -> `key=value` lines, then `END LIST` (terminal use)
    GET <Key>                   -> value of one key
    SET <Key> <value>           -> get/set one key (mode-checked, range-checked, always audited)
    POSE 0|1                    -> start/stop a ~10 Hz `P {json}` lane-geometry stream, fed from
                                   the fork's own driving model by ka2_pose_pub.py
Only keys in KEYS can be read or written, values are range-checked, and every change is appended
to /data/hermes/bt_audit.log (never /var/log: the kommu user cannot write there).

Two truths this file encodes, both learned from the fork's own source rather than assumed:
 * A key written by a daemon is an *output*, not a control. NetworkMetered is rewritten by
   hardwared, DrivePathOffset is clamped and rewritten by modeld, UpdaterTargetBranch is written
   back by updated -- all three are reported `ro`.
 * A key the box deletes at startup cannot be a switch either: selfdrived removes
   ExperimentalMode whenever the car has no openpilot longitudinal, which is this Sealion 7.
"""
import ast
import json
import math
import os
import re
import select
import socket
import subprocess
import threading
import time
import sys
import traceback

try:                                   # only needed to run as the Bluetooth service...
  import dbus
  import dbus.service
  import dbus.mainloop.glib
  from gi.repository import GLib
except ImportError:                    # ...so --selftest stays runnable off the device
  dbus = GLib = None

PARAMS = os.environ.get("KA2_PARAMS_DIR", "/data/params/d")
AUDIT = os.environ.get("KA2_AUDIT", "/data/hermes/bt_audit.log")
POSE_PATH = os.environ.get("KA2_POSE_PATH", "/dev/shm/ka2_pose.json")
TUNING_PATH = os.environ.get("KA2_TUNING_PATH", "/data/hermes/tuning.json")
CONTROLD = os.environ.get("KA2_CONTROLD", "/data/openpilot/selfdrive/controls/controlsd.py")
MEDIA = "/data/media"
SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"
ADAPTER_PATH = "/org/bluez/hci0"
TCP_HOST, TCP_PORT = "127.0.0.1", 9911      # loopback-only: a test path, not a second door
POSE_HOT_S = 2.0                            # a pose older than this is reported as stale

# The lane-correction knobs are not params: the car reads them from a JSON file that controlsd
# re-reads about once a second. The ranges and the shipped defaults are parsed out of the *deployed*
# controlsd.py rather than duplicated here, so the box's own code stays the single source of truth -
# and if that file ever stops reading the tuning file, these rows fall back to "not offered" instead
# of pretending to be live.
TUNING_TITLES = {
  "LANE_CORRECTION_GAIN": ("Lane-centre correction gain",
                           "Live tuning. 0 turns the correction off, i.e. stock lateral behaviour; "
                           "the car's own value is what the deployed branch commits."),
  "LANE_CORRECTION_LOOKAHEAD_S": ("Correction lookahead (s)",
                                  "Live tuning. How far ahead the offset is measured, as a time at "
                                  "the current speed. Longer is gentler."),
  "LANE_CORRECTION_MIN_PROB": ("Minimum lane-line probability",
                               "Live tuning. How confident the model must be in both lane lines "
                               "before the correction acts. Stricter only."),
  "LANE_CORRECTION_MAX_OFFSET_M": ("Maximum lane-centre offset (m)",
                                   "Live tuning. Offsets beyond this are rejected as implausible. "
                                   "Stricter only."),
  "LANE_CORRECTION_MAX_LAT_ACC": ("Maximum extra lateral acceleration (m/s2)",
                                  "Live tuning. The correction's own budget. Can be reduced, never "
                                  "raised past the committed design value."),
  "LANE_CORRECTION_MIN_SPEED": ("Minimum speed (m/s)",
                                "Live tuning. Below this the correction is inactive. Can only be "
                                "raised."),
  "LANE_CORRECTION_FILTER_TAU_S": ("Offset filter time constant (s)",
                                   "Live tuning. Higher smooths the lane offset more; the committed "
                                   "value is the most responsive allowed."),
  "LANE_CORRECTION_MAX_ACC_RATE": ("Rate limit on the correction (m/s2 per s)",
                                   "Live tuning. Lower makes the correction change more slowly. "
                                   "0 means unlimited, as in the code."),
  "LANE_MEMORY_HOLD_S": ("Memory hold after a dropout (s)",
                         "Live tuning. How long a remembered offset survives losing the lane "
                         "lines. Shorter only."),
  "LANE_MEMORY_MAX_YAW_RATE": ("Yaw-rate limit for a held offset (rad/s)",
                               "Live tuning. Above this curvature rate a remembered offset is "
                               "never trusted. Tighter only."),
  # --- the vision -> stock-ACC bridge. These are read by the bridge tool, not by controlsd; it re-reads
  #     the same tuning file about once a second.
  "VIS_TURN_ACC_MIN_SETPOINT_KMH": ("Auto-slow floor (km/h)",
                                    "The bridge slows the car for a bend it sees by stepping the ACC "
                                    "setpoint down 5 km/h at a time. It will never step below this, so "
                                    "this is the floor of the automatic slowing. Can only be raised."),
  "VIS_TURN_ACC_MAX_RESTORE_KMH": ("Auto-raise ceiling (km/h)",
                                   "After the bend the bridge hands the speed back - never above the "
                                   "setpoint you had set yourself, and never above this. Can only be "
                                   "lowered."),
}
# The step each knob moves by when the phone offers +/- buttons rather than a text field, chosen so
# a useful change is a few presses: 0.05 on a 0-1 gain, 0.5 m/s on the 5-20 m/s speed floor, and a
# fine 0.025 rad/s on the yaw-rate gate. Any value in range is still accepted; this only sets the
# size of one press.
TUNING_STEPS = {
  "LANE_CORRECTION_GAIN": 0.05,
  "LANE_CORRECTION_LOOKAHEAD_S": 0.25,
  "LANE_CORRECTION_MIN_PROB": 0.05,
  "LANE_CORRECTION_MAX_OFFSET_M": 0.1,
  "LANE_CORRECTION_MAX_LAT_ACC": 0.05,
  "LANE_CORRECTION_MIN_SPEED": 0.5,
  "LANE_CORRECTION_FILTER_TAU_S": 0.25,
  "LANE_CORRECTION_MAX_ACC_RATE": 0.1,
  "LANE_MEMORY_HOLD_S": 0.05,
  "LANE_MEMORY_MAX_YAW_RATE": 0.025,
  "VIS_TURN_ACC_MIN_SETPOINT_KMH": 5.0,     # one press of the app's + moves the floor a whole ACC step
  "VIS_TURN_ACC_MAX_RESTORE_KMH": 5.0,
}
# Where each live knob lives, and the name of the module-level constant carrying its shipped default.
# controlsd names its constants after the keys; the bridge tool keeps shorter constant names, so its two
# rows are mapped explicitly.
VISION_ACC_TOOL = os.environ.get("KA2_VISION_ACC_TOOL", "/data/hermes/ka2_vision_acc.py")
TUNING_SOURCES = (
  {"path": CONTROLD, "keys": {k: k for k in TUNING_TITLES if k.startswith("LANE_")}},
  {"path": VISION_ACC_TOOL, "keys": {"VIS_TURN_ACC_MIN_SETPOINT_KMH": "MIN_SETPOINT_KMH",
                                     "VIS_TURN_ACC_MAX_RESTORE_KMH": "MAX_RESTORE_KMH"}},
)
_TUNING_CACHE = {"at": 0.0, "limits": {}, "bases": {}, "supported": set()}


def tuning_info(max_age_s=10.0):
  """Every live knob the box's own code reads, gathered from each source file's TUNING_LIMITS.

  Parsed with `ast` (never exec'd) so a service on a car cannot run tuning code by accident. A source counts
  as live only if it actually reads the tuning file, so a build that stopped reading it shows those rows as
  inert rather than pretending they still do something.
  """
  now = time.time()
  if now - _TUNING_CACHE["at"] < max_age_s and _TUNING_CACHE["limits"]:
    return _TUNING_CACHE
  limits, bases, supported = {}, {}, set()
  for source in TUNING_SOURCES:
    try:
      with open(source["path"]) as fh:
        text = fh.read()
      reads_file = "TUNING_PATH" in text and "def read_tuning" in text
      tree = ast.parse(text)
    except Exception as exc:                        # a missing or unparseable source is not fatal
      log("tuning introspection failed for %s: %s" % (source["path"], exc))
      continue
    found = False
    for node in tree.body:
      if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
        name = node.targets[0].id
        for key, const in source["keys"].items():
          if name == const and isinstance(node.value, ast.Constant):
            bases[key] = float(node.value.value)
            found = True
        if name == "TUNING_LIMITS" and isinstance(node.value, ast.Dict):
          for key, value in zip(node.value.keys, node.value.values):
            if isinstance(key, ast.Constant) and isinstance(value, ast.Tuple) and len(value.elts) == 2:
              try:
                limits[key.value] = (float(value.elts[0].value), float(value.elts[1].value))
              except (TypeError, ValueError):
                continue
              found = True
    if reads_file and found:
      supported |= set(source["keys"])
  limits = {k: v for k, v in limits.items() if k in bases}   # only knobs with a range AND a default
  if not limits:
    limits, bases, supported = {}, {}, set()
  _TUNING_CACHE.update({"at": now, "limits": limits, "bases": bases, "supported": supported})
  return _TUNING_CACHE


controlsd_tuning = tuning_info                      # the name the rest of this file was written against


def read_tuning_file():
  """The tuning overrides as {name: float}; an unreadable file is simply no overrides."""
  try:
    with open(TUNING_PATH) as fh:
      data = json.load(fh)
  except Exception:
    return {}
  if not isinstance(data, dict):
    return {}
  out = {}
  for name, value in data.items():
    try:
      out[name] = float(value)
    except (TypeError, ValueError):
      continue
  return out


def write_tuning_value(name, value):
  """Merge one knob into the tuning file atomically, world-readable for controlsd (kommu)."""
  current = read_tuning_file()
  current[name] = value
  tmp = TUNING_PATH + ".tmp"
  with open(tmp, "w") as fh:
    json.dump(current, fh, indent=2, sort_keys=True)
    fh.write("\n")
  os.chmod(tmp, 0o644)
  os.replace(tmp, TUNING_PATH)
  dot = os.open(TUNING_PATH, os.O_RDONLY)          # make sure the rename is durable
  try:
    os.fsync(dot)
  finally:
    os.close(dot)


def tuning_rows():
  """One key entry per live-tunable knob, or [] when the deployed build does not read the file."""
  info = controlsd_tuning()
  if not info["limits"]:
    return []
  entries = []
  overrides = read_tuning_file()
  for name in sorted(info["limits"]):
    low, high = info["limits"][name]
    base = info["bases"].get(name)
    shown = overrides.get(name, base)
    title, desc = TUNING_TITLES.get(name, (name, ""))
    entries.append({
      "k": name, "type": "num", "src": "json", "sec": "tune",
      "mode": "live" if name in info["supported"] else "inert",
      "min": low, "max": high, "step": TUNING_STEPS.get(name, 0.05), "default": base,
      "title": title, "desc": desc,
    })
  entries.append({
    "k": "ACT_RESET_TUNING", "type": "action", "act": "RESET_TUNING", "mode": "live", "sec": "tune",
    "confirm": 1, "label": "Defaults", "title": "Restore shipped defaults",
    "desc": "Clears every experimental value on this card at once and returns them to the committed "
            "defaults - no need to step each one back by hand. Nothing else on this screen is "
            "touched, and the car picks it up within a second.",
  })
  return entries


# key -> dict(type, mode, section, title, desc, opts/min/max, default, lock_key)
# mode: live = the box reads it and a write sticks; inert = recorded but ignored by this build;
#       ro = the box owns it. Values are read from the param files themselves, never invented, and
#       `default` is only used when the file does not exist (params_keys.h's own default).
KEYS = [
  # --- the vendor app's eight software settings, mirrored -------------------------------
  {"k": "OpenpilotEnabledToggle", "type": "bool", "mode": "live", "sec": "sw",
   "title": "Enable bukapilot", "default": "1",
   "desc": "Use the openpilot-family system for adaptive cruise control and lane keep. "
           "Changing this can restart openpilot while the car is powered on."},
  {"k": "QuietMode", "type": "bool", "mode": "live", "sec": "sw", "default": "0",
   "title": "Quiet Mode", "desc": "Play a sound only for safety-critical alerts."},
  {"k": "IsMetric", "type": "bool", "mode": "live", "sec": "sw", "default": "1",
   "title": "Use Metric System", "desc": "Display speed in km/h instead of mph."},
  {"k": "SshEnabled", "type": "bool", "mode": "live", "sec": "sw", "default": "0",
   "title": "Enable SSH", "desc": "Allow SSH logins to this device."},
  {"k": "IsAlcEnabled", "type": "bool", "mode": "live", "sec": "sw", "default": "0",
   "title": "Enable Assisted Lane Change", "desc": "Let a lane-change request be carried out "
           "by the system once the driver confirms it with the indicator."},
  {"k": "IsLdwEnabled", "type": "bool", "mode": "live", "sec": "sw", "default": "0",
   "title": "Enable Lane Departure Warning", "desc": "Alert to steer back when the car drifts "
           "over a detected lane line while driving over 50 km/h."},
  {"k": "RecordFront", "type": "bool", "mode": "live", "sec": "sw", "default": "0",
   "lock_key": "RecordFrontLock",
   "title": "Record and Upload Driver Camera", "desc": "Upload driver-facing camera data to help "
           "improve the driver monitoring algorithm."},
  {"k": "ExperimentalMode", "type": "bool", "mode": "inert", "sec": "sw", "default": "0",
   "title": "Experimental Mode",
   "desc": "Not offered: selfdrived deletes this key on every start for this car, because the "
           "Sealion 7 has no openpilot longitudinal control. A write cannot hold."},
  # --- a control that is real, but whose value lives in the car's code, not a param -----
  {"k": "LongitudinalPersonality", "type": "enum", "mode": "live", "sec": "sw", "default": "1",
   "opts": ["Aggressive", "Standard", "Relaxed"],
   "title": "Driving Personality", "desc": "How closely the system follows a lead car. Standard "
           "is recommended; the steering-wheel distance button also cycles this."},
  # --- device settings: shown, not offered ---------------------------------------------
  {"k": "CarName", "type": "str", "mode": "ro", "sec": "dev", "title": "Car Name",
   "desc": "Set from the car the device has fingerprinted."},
  {"k": "FeaturesPackage", "type": "str", "mode": "ro", "sec": "dev", "title": "Features Package",
   "desc": "Feature bundle applied by the vendor app."},
  {"k": "GithubUsername", "type": "str", "mode": "ro", "sec": "dev", "title": "SSH Keys",
   "desc": "The GitHub account whose public keys are trusted for SSH. Changing it regenerates "
           "that key list and would drop any key added by hand."},
  {"k": "UpdaterTargetBranch", "type": "pick", "mode": "live", "sec": "dev",
   "title": "Target Branch",
   "desc": "What the updater installs. It re-reads this on its next check while parked and installs "
           "at the next offroad cycle, so switching it does not change the running code until the "
           "device updates."},
  {"k": "GsmApn", "type": "str", "mode": "nextstart", "sec": "dev", "title": "Device APN",
   "desc": "Used when the box configures its modem at startup, so a change applies at the next "
           "boot (a SIM must be present). The box's own network screen can apply it immediately.",
   "pattern": r"[A-Za-z0-9][A-Za-z0-9._-]{0,31}",
   "when": "when the box next configures its modem (at boot)"},
  {"k": "DrivePathOffset", "type": "num", "mode": "nextstart", "sec": "dev", "default": "0.0",
   "min": -0.25, "max": 0.25, "step": 0.05, "title": "Path Skew Offset",
   "desc": "Lateral bias of the model's path in metres, in 0.05 m steps. modeld reads this once at "
           "startup, so a change applies at the next drive rather than the moment you set it.",
   "when": "when the car next starts the model (i.e. the next drive)"},
  # Actions, not settings: each writes the same param the vendor's own screen writes, so the box's
  # existing manager does the work - no subprocess, no root-only path, nothing to reinvent. The reboot
  # is refused only while autodrive is armed (the owner's rule: sitting in a car park with the ignition
  # on and autodrive off is exactly when you want it), the calibration reset while the car is on at all.
  {"k": "ACT_REBOOT", "type": "action", "act": "REBOOT", "mode": "live", "sec": "dev",
   "confirm": 1, "gate": "engaged", "title": "Reboot the box",
   "desc": "Reboots the device the way the vendor app does. Refused only while autodrive is armed - "
           "the car simply being on is fine. It comes back in about a minute and this link drops."},
  {"k": "ACT_RESET_CALIBRATION", "type": "action", "act": "RESET_CALIBRATION", "mode": "live",
   "sec": "dev", "confirm": 1, "gate": "parked", "title": "Reset Calibration",
   "desc": "Clears the learned camera and live-steering calibration; the device recalibrates as you "
           "drive. Refused while the car is on."},
  # (NetworkMetered was removed at the owner's request: hardwared rewrites it, a hotspot counts as
  # Wi-Fi, and a row that can never hold is clutter on a settings screen.)
]
BY_KEY = {e["k"]: e for e in KEYS}


def entry_for(key):
  """The declared entry for a key, including the tuning knobs read from the deployed controlsd."""
  entry = BY_KEY.get(key)
  if entry is not None:
    return entry
  for candidate in tuning_rows():
    if candidate["k"] == key:
      return candidate
  return None


def all_entries():
  return KEYS + tuning_rows()

BOOL_TRUE = {"1", "true", "on", "yes", "enable", "enabled"}
BOOL_FALSE = {"0", "false", "off", "no", "disable", "disabled"}

if AUDIT:
  os.makedirs(os.path.dirname(AUDIT), exist_ok=True)


def log(msg):
  line = "%s %s" % (time.strftime("%Y-%m-%dT%H:%M:%S"), msg)
  print(line, flush=True)
  if AUDIT:
    try:
      with open(AUDIT, "a") as fh:
        fh.write(line + "\n")
    except Exception:
      pass


def read_key(key):
  try:
    with open(os.path.join(PARAMS, key)) as fh:
      return fh.read().strip()
  except Exception:
    return None


try:                                    # Wi-Fi lives in its own module; a missing copy is not fatal
  from wifi_control import redact as wifi_redact, wifi_reply as _wifi_reply
  WIFI_AVAILABLE = True
except Exception as _wifi_exc:          # pragma: no cover - only on a bad deploy
  WIFI_AVAILABLE = False
  _WIFI_ERROR = str(_wifi_exc)

  def wifi_redact(line):
    return line

  def _wifi_reply(command, payload):
    return ["ERR WIFI control unavailable on the box (%s)" % _WIFI_ERROR]


def car_state():
  """What the car is doing, from the box's own params - the source of the app's state line."""
  def flag(name):
    return read_key(name) in BOOL_TRUE
  return {"onroad": flag("IsOnroad"), "engaged": flag("IsEngaged"),
          "offroad": flag("IsOffroad"), "controls_ready": flag("ControlsReady")}


def state_line():
  return "S " + json.dumps(car_state(), separators=(",", ":"))


def read_or_default(entry):
  value = read_key(entry["k"])
  if value is None or value == "":
    return entry.get("default", "")
  return value


def typed_ok(entry, raw):
  """Whether the stored text can be read back as the type the fork declares for that key.

  Worth the few lines: the first version of the phone app wrote `LongitudinalPersonality` as
  "2.0", the fork declares it INT, so the cast failed and `get(..., return_default=True)` handed
  the car the *default* (Standard) - the owner's "Relaxed" choice was silently gone. A value the
  box cannot read is worse than an absent one, so it is reported rather than displayed as truth.
  """
  if raw is None or raw == "":
    return False
  kind = entry["type"]
  if kind == "bool":
    return raw in ("0", "1")
  if kind == "num":
    try:
      float(raw)
      return True
    except ValueError:
      return False
  if kind == "enum":
    try:
      return 0 <= int(raw) < len(entry.get("opts", []))
    except ValueError:
      return False
  return True


def effective(entry):
  """(value the box will actually use, stored-but-unreadable text or None)."""
  if entry.get("src") == "json":
    overrides = read_tuning_file()
    name = entry["k"]
    if name in overrides:
      return "%g" % overrides[name], None
    return "%g" % entry.get("default", 0.0), None
  raw = read_key(entry["k"])
  if raw is None or raw == "":
    return entry.get("default", ""), None
  if not typed_ok(entry, raw):
    return entry.get("default", ""), raw
  return raw, None


def mode_of(entry):
  """`live` unless the box locks the key, in which case the phone must not offer it either.

  Tuning knobs are re-checked on every read: they are live only while the *deployed* controlsd is
  actually reading the tuning file, so a build that stops honouring it cannot leave the phone
  offering a knob the car ignores.
  """
  if entry.get("src") == "json":
    return "live" if controlsd_tuning()["supported"] else "inert"
  lock = entry.get("lock_key")
  if lock and str(read_or_default({"k": lock, "default": "0"})).lower() in BOOL_TRUE:
    return "ro"
  return entry["mode"]


def write_param(key, text):
  try:
    with open(os.path.join(PARAMS, key), "w") as fh:
      fh.write(text)
    return True
  except Exception as exc:
    log("write %s failed: %s" % (key, exc))
    return False


def grid_range(entry):
  """The values a stepped key can actually take: whole steps that lie inside its limits.

  Why this exists: a ceiling is not always on the step grid. LANE_CORRECTION_MIN_PROB is capped at
  0.99 with a 0.05 step, so its top usable value is 0.95 - and a client that clamped to 0.99 and
  sent it got a refusal, because the value was snapped to the grid *before* the range check. The
  advertised range is therefore the usable one, and writes are snapped into it rather than rejected.
  """
  low, high = entry.get("min"), entry.get("max")
  step = entry.get("step")
  if low is None or high is None or not step or entry.get("type") != "num":
    return low, high
  first = math.ceil(low / step - 1e-9)
  last = math.floor(high / step + 1e-9)
  if last < first:                       # narrower than one step: keep the limits as given
    return low, high
  return round(first * step, 6), round(last * step, 6)


def normalise(entry, raw):
  """Validate a write; returns (value_to_write, error_message, note).

  `note` is set when the stored value differs from the request for a benign reason - a value between
  two steps is snapped onto the grid - so the client is told rather than left guessing.
  """
  kind = entry["type"]
  if kind == "bool":
    token = raw.strip().lower()
    if token not in BOOL_TRUE and token not in BOOL_FALSE:
      return None, "ERR not-a-boolean (use 1/0)", None
    return ("1" if token in BOOL_TRUE else "0"), None, None
  if kind == "num":
    try:
      requested = float(raw)
    except ValueError:
      return None, "ERR not-a-number", None
    lo, hi = entry.get("min"), entry.get("max")
    if lo is not None and requested < lo - 1e-9:
      return None, "ERR out-of-range %s..%s" % (lo, hi), None
    if hi is not None and requested > hi + 1e-9:
      return None, "ERR out-of-range %s..%s" % (lo, hi), None
    # Inside the limits, land on the usable grid instead of refusing: a ceiling need not be a whole
    # number of steps (MIN_PROB is capped at 0.99 with a 0.05 step), and a refusal there is exactly
    # the dead end the owner reported as "the value is no longer settable at the maximum".
    original = requested
    usable_lo, usable_hi = grid_range(entry)
    if usable_lo is not None and usable_hi is not None:
      requested = min(usable_hi, max(usable_lo, requested))
    step = entry.get("step")
    value = round(round(requested / step) * step, 6) if step else requested
    note = None
    if step and abs(value - original) > 1e-9:
      note = "snapped onto the %s step grid" % ("%g" % step)
    if lo is not None and value < lo - 1e-9:
      return None, "ERR out-of-range %s..%s" % (lo, hi), None
    if hi is not None and value > hi + 1e-9:
      return None, "ERR out-of-range %s..%s" % (lo, hi), None
    if entry.get("src") == "json":
      return value, None, note                     # a number, written into the tuning JSON
    if step and step < 0.01:                       # modeld stores this one as text: "0.0" / "%.2f"
      return ("0.0" if value == 0.0 else "%.2f" % value), None, note
    return repr(round(value, 4)), None, note
  if kind == "enum":
    try:
      index = int(float(raw))
    except ValueError:
      # accept the option's name too, so a client can send what it displays
      lowered = [o.lower() for o in entry.get("opts", [])]
      if raw.strip().lower() in lowered:
        return str(lowered.index(raw.strip().lower())), None, None
      return None, "ERR not-an-option", None
    if not 0 <= index < len(entry.get("opts", [])):
      return None, "ERR out-of-range 0..%d" % (len(entry.get("opts", [])) - 1), None
    return str(index), None, None
  text = raw.strip()
  pattern = entry.get("pattern")
  if pattern and not re.fullmatch(pattern, text):
    return None, "ERR bad-value (letters, digits, dot or dash; no spaces)", None
  if not re.fullmatch(r"[A-Za-z0-9_. +-]{0,64}", text):
    return None, "ERR bad-value", None
  return text, None, None


def write_key(key, raw):
  entry = entry_for(key)
  if entry is not None and entry.get("type") == "action":
    return run_action(entry)
  if key == "UpdaterTargetBranch":
    wanted = raw.strip()
    options = available_branches()
    if wanted not in options:
      return "ERR not-an-available-branch (%s)" % (",".join(options) if options else "none listed")
    if not write_param(key, wanted):
      return "ERR write-failed"
    log("SET %s = %s" % (key, wanted))
    return "OK %s=%s mode=live (installs at the next offroad cycle)" % (key, wanted)
  if entry is None:
    return "ERR not-whitelisted"
  mode = mode_of(entry)
  if mode not in ("live", "nextstart"):
    return "ERR %s is %s in this build - not writable" % (key, mode)
  value, err, note = normalise(entry, raw)
  if err:
    return err
  try:
    if entry.get("src") == "json":
      write_tuning_value(key, float(value))
    else:
      with open(os.path.join(PARAMS, key), "w") as fh:
        fh.write(value)
  except Exception as exc:
    return "ERR write-failed %s" % exc
  # Read it back: on this box a write that lands is not proof the setting held (a daemon may
  # rewrite it a moment later), so the reply carries what the param file actually says now.
  log("SET %s = %s" % (key, value))
  if entry.get("src") == "json":
    return "OK %s=%s mode=live%s" % (key, effective(entry)[0], " (%s)" % note if note else "")
  if mode == "nextstart":
    return "OK %s=%s mode=nextstart (%sapplies %s)" % (
        key, effective(entry)[0], "%s; " % note if note else "",
        entry.get("when", "at the next start of the component that reads it"))
  # Read back through the same validation the box's own reader applies, so the reply cannot claim
  # success for a write the car would not be able to cast.
  stored, invalid = effective(entry)
  if invalid is not None:
    return "ERR %s written but unreadable (%s)" % (key, invalid)
  if note:
    return "OK %s=%s mode=%s (%s)" % (key, stored, mode, note)
  return "OK %s=%s mode=live" % (key, stored)


# --- device facts ---------------------------------------------------------------------

def _first_line(path, default=""):
  try:
    with open(path) as fh:
      return fh.readline().strip() or default
  except Exception:
    return default


def _ip_of(iface):
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    import struct
    import fcntl
    packed = fcntl.ioctl(s.fileno(), 0x8915, struct.pack("256s", iface[:15].encode()))
    return socket.inet_ntoa(packed[20:24])
  except Exception:
    return ""


def _ssid():
  try:
    out = subprocess.run(["iw", "dev", "wlan0", "link"], capture_output=True, text=True,
                         timeout=3).stdout
    match = re.search(r"SSID:\s*(\S.*)", out)
    return match.group(1).strip() if match else ""
  except Exception:
    return ""


def device_info():
  """Everything the phone's Connected Device card shows, read from the box itself."""
  onroad = read_or_default({"k": "IsOnroad", "default": "0"}) in BOOL_TRUE
  nets = []
  for iface, label in (("wlan0", "wifi"), ("wwan0", "cellular"), ("usb0", "usb")):
    ip = _ip_of(iface)
    if ip:
      nets.append({"if": iface, "kind": label, "ip": ip,
                   "up": _first_line("/sys/class/net/%s/operstate" % iface, "unknown")})
  primary = nets[0] if nets else {"if": "", "kind": "none", "ip": "", "up": ""}
  free_gb = None
  try:
    stat = os.statvfs(MEDIA)
    free_gb = round(stat.f_bavail * stat.f_frsize / 1e9, 1)
  except Exception:
    pass
  uptime = 0.0
  try:
    uptime = round(float(_first_line("/proc/uptime", "0").split()[0]), 0)
  except Exception:
    pass
  pose = read_pose()
  return {
    "dongle": read_or_default({"k": "DongleId"}),
    "serial": read_or_default({"k": "HardwareSerial"}),
    "version": read_or_default({"k": "Version"}),
    "commit": read_or_default({"k": "GitCommit"})[:8],
    "branch": read_or_default({"k": "GitBranch"}),
    "desc": read_or_default({"k": "UpdaterCurrentDescription"}),
    "state": "onroad" if onroad else "offroad",
    "car": read_or_default({"k": "CarName"}),
    "net": primary["kind"], "ssid": _ssid() if primary["kind"] == "wifi" else "",
    "ip": primary["ip"], "iface": primary["if"], "link": primary["up"],
    "metered": read_or_default({"k": "NetworkMetered", "default": "0"}),
    "free_gb": free_gb, "uptime_s": uptime,
    "pose": {"ok": pose.get("ok", 0), "why": pose.get("why", ""), "age": pose.get("age")},
  }


def read_pose():
  """The newest pose from the publisher, with its own age - a stale file is reported as such."""
  try:
    with open(POSE_PATH) as fh:
      pose = json.load(fh)
  except Exception as exc:
    return {"ok": 0, "why": "pose publisher not running (%s)" % type(exc).__name__, "age": None}
  age = round(time.time() - float(pose.get("t", 0)), 2)
  pose["age"] = age
  if age > POSE_HOT_S:
    pose["ok"] = 0
    pose["why"] = "pose publisher stale (%.1fs)" % age
  return pose


# --- the line protocol ----------------------------------------------------------------

def available_branches():
  """The branches the updater says it can fetch (param written by updated itself)."""
  raw = read_key("UpdaterAvailableBranches") or ""
  return [b.strip() for b in raw.split(",") if b.strip()]


def run_action(entry):
  """Run one action through the box's own mechanism.

  Gates are per action, because "the car is on" is not the same as "the ADAS is driving":

  * ``gate: "engaged"`` (reboot) refuses only while the ADAS is armed or engaged. The owner pointed
    out that the old rule - refuse whenever the car was on - blocked a reboot in a car park with the
    ignition on and autodrive off, which is the case he actually hits. ``IsEngaged`` is written by
    hardwared from ``selfdriveState.enabled``, i.e. exactly "autodrive is armed".
  * ``gate: "parked"`` (calibration reset) still refuses while the car is on, since it throws away
    learned calibration and expects a drive to relearn it.
  * No gate: reverting tuning values is the same class of change as moving a slider, and the point of
    the tuning file is that it applies live.
  """
  onroad = (read_key("IsOnroad") or "0").strip().lower() in BOOL_TRUE
  armed = (read_key("IsEngaged") or "0").strip().lower() in BOOL_TRUE
  if entry.get("gate") == "engaged" and armed:
    return ("ERR the ADAS is armed (IsEngaged=1) - %s refused while it could take over; turn "
            "autodrive off first" % entry["act"].lower())
  if entry.get("gate") == "parked" and onroad:
    return "ERR car is on (onroad) - %s refused until it is parked" % entry["act"].lower()
  if entry["act"] == "REBOOT":
    if not write_param("DoReboot", "1"):
      return "ERR write-failed"
    log("ACT REBOOT")
    return "OK ACT_REBOOT=queued (the box drops off for about a minute; this link will close)"
  if entry["act"] == "RESET_CALIBRATION":
    removed = []
    for key in ("CalibrationParams", "LiveTorqueParameters", "LiveParameters", "LiveParametersV2",
                "LiveDelay"):
      try:
        os.remove(os.path.join(PARAMS, key))
        removed.append(key)
      except OSError:
        pass
    write_param("OnroadCycleRequested", "1")
    log("ACT RESET_CALIBRATION removed=%s" % ",".join(removed))
    return "OK ACT_RESET_CALIBRATION=%d/5 (cleared the stored calibration; it recalibrates as you drive)" % len(removed)
  if entry["act"] == "RESET_TUNING":
    cleared = sorted(read_tuning_file())
    try:
      os.remove(TUNING_PATH)
    except FileNotFoundError:
      pass
    except OSError as exc:
      return "ERR could not clear the tuning file: %s" % exc
    log("ACT RESET_TUNING cleared=%s" % (",".join(cleared) if cleared else "none"))
    if not cleared:
      return "OK ACT_RESET_TUNING=0 (already on the shipped defaults - nothing was overridden)"
    return "OK ACT_RESET_TUNING=%d (%d value%s restored to the committed defaults: %s)" % (
        len(cleared), len(cleared), "" if len(cleared) == 1 else "s", ", ".join(cleared))
  return "ERR unknown-action"


def schema_rows():
  rows = []
  for entry in all_entries():
    mode = mode_of(entry)
    value, invalid = effective(entry)
    row = {"k": entry["k"], "t": entry["type"], "sec": entry["sec"], "mode": mode,
           "v": value, "title": entry["title"], "desc": entry["desc"]}
    if invalid is not None:
      row["warn"] = "stored value %r cannot be read as %s; the box uses %s" % (
          invalid, entry["type"], value)
    if entry.get("opts"):
      row["opts"] = entry["opts"]
    if entry.get("min") is not None:
      row["min"] = entry["min"]
    if entry.get("max") is not None:
      row["max"] = entry["max"]
    usable_lo, usable_hi = grid_range(entry)
    if usable_lo is not None and usable_hi is not None and (usable_lo != entry.get("min")
                                                            or usable_hi != entry.get("max")):
      row["min"], row["max"] = usable_lo, usable_hi     # the range the client can actually reach
    if entry.get("step") is not None:
      row["step"] = entry["step"]      # numeric rows are quantised: say so, so the app can hint it
    if entry.get("confirm"):
      row["confirm"] = 1               # the client must ask before sending this one
    if entry.get("act"):
      row["act"] = entry["act"]
    if entry.get("gate"):
      row["gate"] = entry["gate"]      # the client words its confirmation to match the gate
    if entry.get("label"):
      row["label"] = entry["label"]
    if entry["k"] == "UpdaterTargetBranch":
      options = available_branches()
      if options:
        row["opts"] = options
      row["v"] = read_or_default(entry)
    if mode == entry["mode"] and mode != "live":
      row["reason"] = entry["desc"]
    rows.append(row)
  return rows


ACC_PY = "/usr/local/venv/bin/python3"     # the interpreter that has capnp/cereal (system python3 lacks it)
ACC_TOOL = "/data/hermes/ka2_acc_press.py"
ACC_CWD = "/data/openpilot"
ACC_USER = "kommu"                         # owns openpilot's /dev/shm msgq files; root cannot read them
ACC_REQUESTS = {"UP": "step", "DOWN": "set", "CANCEL": "cancel",
                "DIST+": "inc_dist", "DIST-": "dec_dist",
                # explicit patterns, for working out which one this car actually accepts. Measured so far
                # (Sealion 7, ACC engaged): SET alone steps the setpoint DOWN (45 -> 40 observed);
                # RES alone CANCELS the ACC rather than increasing it. "step" is the car's own speed-rocker
                # pattern (SET+RES together) and is what UP now sends.
                "RES": "res", "SET": "set", "STEP": "step"}


def acc_reply(sub):
  """One line back to the app: the car's ACC state, a press result, or why a press was refused.

  The press tool owns every gate - ACC must be switched on, minimum gap between presses, never
  LKAS_ON, one button bit per frame, one audit line per press - and this service only relays. It runs
  out of process because this service is on /usr/bin/python3, which has no capnp/cereal: one place for
  the rules, one interpreter that can actually reach the bus.
  """
  sub = (sub or "STATE").upper()
  if sub in ("STATE", ""):
    cmd = [ACC_PY, ACC_TOOL, "--state"]
  else:
    button = ACC_REQUESTS.get(sub)
    if button is None:
      return "ACC unknown request '%s' (have STATE, %s)" % (sub, ", ".join(sorted(ACC_REQUESTS)))
    cmd = [ACC_PY, ACC_TOOL, "--button", button]
  try:
    # Run as kommu, not as root: openpilot's msgq files in /dev/shm are owned by kommu and root cannot
    # read msgq_carState ("could not open: /dev/shm/msgq_carState: Permission denied"), while kommu
    # reads it fine. Same interpreter the box's other publisher scripts use.
    p = subprocess.run(cmd, cwd=ACC_CWD, capture_output=True, text=True, timeout=10,
                       user=ACC_USER, group=ACC_USER)
  except Exception as exc:
    return "ACC press tool did not run: %s" % str(exc)[:120]
  out = [ln for ln in (p.stdout or "").strip().splitlines() if ln.strip()]
  if out:
    return "ACC " + out[-1].strip()
  err = [ln for ln in (p.stderr or "").strip().splitlines() if ln.strip()]
  return "ACC press tool said nothing (rc=%s%s)" % (p.returncode, (": " + err[-1][:100]) if err else "")


def handle(line):
  parts = line.strip().split()
  if not parts:
    return []
  cmd = parts[0].upper()
  if cmd == "HELP":
    return ["OK commands: INFO | SCHEMA | LIST | GET <Key> | SET <Key> <value> | ACT <NAME> | "
            "POSE 0|1 | VER <client> | HELP"]
  if cmd == "WIFI":
    # The payload is everything after the subcommand, unsplit: a password may contain spaces.
    pieces = line.split(None, 2)
    sub = pieces[1].upper() if len(pieces) > 1 else "STATUS"
    payload = None
    if len(pieces) > 2:
      try:
        payload = json.loads(pieces[2])
      except Exception:
        return ["ERR WIFI bad-json-payload"]
    if not WIFI_AVAILABLE:
      return _wifi_reply(sub, payload)
    return _wifi_reply(sub, payload, _ip_of)
  if cmd.startswith("DRIVES"):
    return _log_command(cmd)

  if cmd == "STATE":
    return [state_line()]
  if cmd == "INFO":
    return ["I " + json.dumps(device_info(), separators=(",", ":"))]
  if cmd == "SCHEMA":
    rows = schema_rows()
    out = ["K " + json.dumps(row, separators=(",", ":")) for row in rows]
    out.append("E " + json.dumps({"n": len(rows)}, separators=(",", ":")))
    return out
  if cmd == "LIST":
    out = ["%s=%s" % (e["k"], effective(e)[0]) for e in all_entries()]
    out.append("END LIST")
    return out
  if cmd == "GET" and len(parts) == 2:
    entry = entry_for(parts[1])
    if entry is None:
      return ["ERR not-whitelisted"]
    value, invalid = effective(entry)
    note = " invalid=%s" % invalid if invalid is not None else ""
    return ["%s=%s mode=%s%s" % (entry["k"], value, mode_of(entry), note)]
  if cmd == "SET" and len(parts) >= 3:
    return [write_key(parts[1], " ".join(parts[2:]))]
  if cmd == "VER":
    # The client names its build on connect. Cheap, and it removes the guessing: the audit log now
    # says which app version produced a sequence of writes.
    log("APP %s" % " ".join(parts[1:])[:40])
    return ["OK ver"]
  if cmd == "ACT" and len(parts) == 2:
    entry = entry_for("ACT_" + parts[1].upper().replace("-", "_"))
    if entry is None or entry.get("act") is None:
      return ["ERR unknown-action"]
    if mode_of(entry) not in ("live", "nextstart"):
      return ["ERR action %s is not available in this build" % parts[1]]
    return [run_action(entry)]
  if cmd == "ACC":
    # Stock ACC buttons (0x3B0): a press, not a setting. It acts on the car immediately, so it stays
    # out of the SET/ACT schema - and the tool it calls never touches LKAS_ON_BTN.
    return [acc_reply(parts[1] if len(parts) > 1 else "STATE")]
  return ["ERR usage"]


_LAST_REFUSAL = {"key": "", "at": 0.0}


def log_refusal(command, reply):
  """Record a refused command, at most one line every two seconds per distinct refusal.

  A client that gets stuck retrying would otherwise bury the reason under thousands of identical
  lines - which is exactly what made one debugging round take longer than it needed to.
  """
  stamp = "%s|%s" % (command, reply)
  now = time.time()
  if _LAST_REFUSAL["key"] == stamp and now - _LAST_REFUSAL["at"] < 2.0:
    return
  _LAST_REFUSAL["key"] = stamp
  _LAST_REFUSAL["at"] = now
  log("REFUSED %s -> %s" % (command[:60], reply[:80]))


class Session:
  """One connected client: command/reply plus an optional pose stream.

  The recv timeout is the multiplexer - no second thread, so a pose can never interleave with a
  reply on the same socket.
  """

  KEEPALIVE_S = 5.0       # an idle RFCOMM link is what phone Bluetooth stacks drop
  STATE_EVERY_S = 1.0     # how often the car's state is re-read and, if changed, pushed
  READ_TIMEOUT_S = 0.1    # how long a read waits before the pose loop takes a turn
  SEND_TIMEOUT_S = 5.0    # a write gets far longer: a Bluetooth link that is briefly busy must not
                          # kill the session, which is exactly what a 0.1 s write timeout did

  def __init__(self, sock, label):
    self.sock = sock
    self.label = label
    self.pose = False
    self.buf = b""
    self.opening = 0        # how many of this session's commands have been logged
    self.sent_at = time.time()
    self.last_state = None
    self.state_at = 0.0

  def send_line(self, text):
    # settimeout applies to writes as well as reads, so the short read timeout must be put back after
    # every send - otherwise a momentary pause in the link aborts the connection.
    self.sock.settimeout(self.SEND_TIMEOUT_S)
    try:
      self.sock.sendall((text + "\n").encode())
    finally:
      self.sock.settimeout(self.READ_TIMEOUT_S)

  def try_send_pose(self, text):
    """A pose that the link cannot take now is dropped, not queued: the next one is fresher anyway."""
    try:
      if not select.select([], [self.sock], [], 0)[1]:
        return False
      self.send_line(text)
      return True
    except (socket.timeout, OSError):
      return False

  def run(self):
    self.sock.settimeout(self.READ_TIMEOUT_S)
    self.send_line("KA2 SETTINGS v2")
    announced = state_line()             # so the app shows the state from the moment it connects
    self.send_line(announced)
    self.last_state = announced          # and the change detector does not repeat it a moment later
    log("connect %s" % self.label)
    try:
      while True:
        try:
          chunk = self.sock.recv(1024)
        except socket.timeout:
          now = time.time()
          if self.pose:
            self.try_send_pose("P " + json.dumps(read_pose(), separators=(",", ":")))
          elif now - self.sent_at > self.KEEPALIVE_S:
            # Nothing to say for five seconds: say so anyway. A phone's Bluetooth stack treats a silent
            # RFCOMM link as dead and drops it, which showed up as connections that kept resetting.
            self.sent_at = now
            self.send_line("# keepalive")
          # The state check is not an "else" of the two above. It used to be, so with the lane page's
          # pose stream running (always) or a keepalive due, the state went out once at connect and
          # never again - and the app's status line sat on whatever it read first.
          if now - self.state_at > self.STATE_EVERY_S:
            self.state_at = now
            state = state_line()
            if state != self.last_state:   # onroad / engaged changed: tell the app at once
              self.last_state = state
              self.send_line(state)
          continue                          # a read timeout is not a message: go round again
        if not chunk:
          break
        self.buf += chunk
        if len(self.buf) > 8192:                      # a client that never sends a newline
          self.send_line("ERR line-too-long")
          self.buf = b""
          continue
        while b"\n" in self.buf:
          raw, self.buf = self.buf.split(b"\n", 1)
          line = raw.decode("utf-8", "replace")
          command = line.strip().split()[0].upper() if line.strip() else ""
          if self.opening < 8:
            # The first commands of a session, so a link that dies straight after connecting can be
            # read off the box rather than guessed at from the phone end.
            self.opening += 1
            log("cmd %s: %s" % (self.label, wifi_redact(line)[:60]))
          if command == "POSE":
            arg = line.strip().split()[1] if len(line.strip().split()) > 1 else ""
            self.pose = arg.strip() in ("1", "on", "true")
            self.send_line("OK pose-%s" % ("on" if self.pose else "off"))
            log("pose %s %s" % ("on" if self.pose else "off", self.label))
            continue
          for reply in handle(line):
            if reply:
              self.send_line(reply)
              if reply.startswith("ERR") and command not in ("", "POSE"):
                log_refusal(command, reply)
    except Exception as exc:
      # Name the fault's own line: an opaque "Permission denied" with no location cost a whole round
      # trip of guessing which of the commands in this loop raised it.
      frames = traceback.extract_tb(sys.exc_info()[2])
      where = "%s:%d" % (frames[-1].filename.split("/")[-1], frames[-1].lineno) if frames else "?"
      log("session ended: %s: %s at %s" % (type(exc).__name__, exc, where))
    finally:
      try:
        self.sock.close()
      except Exception:
        pass
      log("disconnect %s" % self.label)


def serve_tcp():
  """Loopback-only twin of the Bluetooth service: lets us exercise the protocol over ssh
  (`ssh -L 9911:127.0.0.1:9911`) with the phone out of the picture."""
  srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  srv.bind((TCP_HOST, TCP_PORT))
  srv.listen(4)
  log("loopback test listener on %s:%d" % (TCP_HOST, TCP_PORT))
  while True:
    try:
      conn, addr = srv.accept()
      Session(conn, "tcp %s:%d" % addr).run()
    except Exception as exc:
      log("tcp listener error: %s" % exc)


class Profile(dbus.service.Object):
  @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}", out_signature="")
  def NewConnection(self, device, fd, props):
    # BlueZ waits for this call to return, while the session lasts as long as the phone stays
    # connected. Running it inline meant bluetoothd timed out waiting for a reply - its log read
    # "KA2 Settings replied with an error: NoReply" - and dropped the link about 25 seconds in, over
    # and over. The session belongs on its own thread.
    serve_in_background(socket.socket(fileno=fd.take()), "bt %s" % device)

  @dbus.service.method("org.bluez.Profile1", in_signature="", out_signature="")
  def Release(self):
    log("release")

  @dbus.service.method("org.bluez.Profile1", in_signature="o", out_signature="")
  def RequestDisconnection(self, device):
    log("request-disconnection %s" % device)


AGENT_PATH = "/com/hermes/ka2settings/agent"


class Agent(dbus.service.Object):
  """Pairing agent. NoInputNoOutput = this box has no keypad or display, which is what a
  headless device must advertise; pairing then completes without a prompt on the box side."""

  @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
  def Release(self):
    log("agent: release")

  @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="s")
  def RequestPinCode(self, device):
    return "0000"

  @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="u")
  def RequestPasskey(self, device):
    return dbus.UInt32(0)

  @dbus.service.method("org.bluez.Agent1", in_signature="ouq", out_signature="")
  def DisplayPasskey(self, device, passkey, entered):
    log("agent: display passkey")

  @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
  def DisplayPinCode(self, device, pincode):
    log("agent: display pin")

  @dbus.service.method("org.bluez.Agent1", in_signature="ou", out_signature="")
  def RequestConfirmation(self, device, passkey):
    log("agent: confirmed %s" % device)

  @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="")
  def RequestAuthorization(self, device):
    log("agent: authorized %s" % device)

  @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
  def AuthorizeService(self, device, uuid):
    log("agent: authorized service %s" % uuid)

  @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
  def Cancel(self):
    log("agent: cancel")


def selftest():
  """Exercise every command against the real (or a test) params directory, no Bluetooth needed.
  Prints exactly what a client would receive, so the app's parser can be tested against it."""
  for entry in KEYS:
    for line in handle("GET %s" % entry["k"]):
      print(line)
  for line in handle("INFO"):
    print(line)
  for line in handle("SCHEMA"):
    print(line)
  print(handle("SET QuietMode on")[0])
  print(handle("SET QuietMode 1")[0])
  print(handle("SET ExperimentalMode 1")[0])
  print(handle("SET NetworkMetered 1")[0])
  print(handle("SET IsMetric bogus")[0])
  print(handle("SET LongitudinalPersonality 2")[0])
  print(handle("SET LongitudinalPersonality 9")[0])
  print(handle("SET LongitudinalPersonality relaxed")[0])
  for name in sorted(controlsd_tuning()["limits"]):
    step = TUNING_STEPS.get(name, 0.05)
    print(handle("GET %s" % name)[0] + "   [step %s]" % step)
    print("  one press down -> " + handle("SET %s %s" % (name, float(effective(entry_for(name))[0]) - step))[0])
    print("  one press up   -> " + handle("SET %s %s" % (name, float(effective(entry_for(name))[0]) + step))[0])
    print("  " + handle("GET %s" % name)[0])
  print(handle("NONSENSE")[0])
  return 0


def serve_in_background(sock, label):
  """Run one client's session on its own thread, and return at once.

  Called from inside a D-Bus method (BlueZ's Profile1.NewConnection), so it must not block: the reply
  is what tells BlueZ the connection was accepted.
  """
  thread = threading.Thread(target=Session(sock, label).run, daemon=True)
  thread.start()
  return thread


def profile_options():
  """How the SPP service is registered with BlueZ.

  Both flags must stay false, and the second one is the one that took a Bluetooth capture to find.

  RequireAuthentication asks BlueZ for an authenticated (bonded) link even though the app connects
  without pairing. RequireAuthorization looks harmless - the NoInputNoOutput agent answers the
  authorisation silently - but on classic Bluetooth an authorised link is a *secure* link, so BlueZ
  still needs a bond to satisfy it. With either flag set, BlueZ starts the pairing itself: an HCI
  capture showed a Secure Simple Pairing exchange initiated with this service's own "No bonding, No
  MITM, No Keypresses" requirements, a User Confirmation Request pushed to the phone (Android's
  "Pair with kommu-...?" dialog), and a link key created.

  Neither is needed here: the app is the only client, it connects unauthenticated on purpose, and
  nothing is asked of the user either way.
  """
  return {"Name": "KA2 Settings", "Role": "server", "Channel": 1,
          "RequireAuthentication": False, "RequireAuthorization": False,
          "Service": SPP_UUID}


def main():
  if "--selftest" in sys.argv:
    return selftest()
  dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
  bus = dbus.SystemBus()
  path = "/com/hermes/ka2settings"
  profile = Profile(bus, path)
  try:
    mgr = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"), "org.bluez.ProfileManager1")
    mgr.RegisterProfile(path, SPP_UUID, profile_options())
    log("registered SPP profile at %s" % path)
  except Exception as exc:
    log("REGISTER FAILED: %s" % exc)
    return 1
  agent_obj = Agent(bus, AGENT_PATH)
  try:
    amgr = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"), "org.bluez.AgentManager1")
    amgr.RegisterAgent(AGENT_PATH, "NoInputNoOutput")
    amgr.RequestDefaultAgent(AGENT_PATH)
    log("pairing agent registered (NoInputNoOutput, default)")
  except Exception as exc:
    log("AGENT FAILED: %s" % exc)
  # Adapter state, re-applied on every start: a discoverable timeout of 180 s (the default)
  # is why the box kept disappearing from scans, and hciconfig's class does not survive a
  # bluetoothd restart. Setting them here means the unit owns them.
  try:
    props = dbus.Interface(bus.get_object("org.bluez", ADAPTER_PATH),
                           "org.freedesktop.DBus.Properties")
    for name, value in (("Powered", dbus.Boolean(True)), ("Discoverable", dbus.Boolean(True)),
                        ("Pairable", dbus.Boolean(True)), ("DiscoverableTimeout", dbus.UInt32(0))):
      props.Set("org.bluez.Adapter1", name, value)
    log("adapter: discoverable, pairable, timeout 0")
  except Exception as exc:
    log("ADAPTER FAILED: %s" % exc)
  threading.Thread(target=serve_tcp, daemon=True).start()
  GLib.MainLoop().run()
  return 0


def _log_command(cmd):
  """LOG LIST / LOG SUMMARY <route>: drive summaries for the app's log page.

  Reading a drive's logs takes seconds, so it happens here and is cached; the reply is one line of
  compact JSON, which is what the protocol expects.
  """
  import subprocess
  parts = cmd.split()
  args = ["summary", parts[1]] if len(parts) >= 2 else ["list"]
  env = dict(os.environ, PYTHONPATH="/data/openpilot")
  try:
    r = subprocess.run(["/usr/local/venv/bin/python3", "/data/hermes/log_summary.py"] + args,
                       capture_output=True, text=True, timeout=120, env=env)
  except Exception as exc:
    return ["DRIVES ERROR %s" % exc]
  if r.returncode != 0:
    return ["DRIVES ERROR " + (r.stderr.strip().splitlines() or ["failed"])[-1][:180]]
  return ["DRIVES " + " ".join(r.stdout.split())]



if __name__ == "__main__":
  sys.exit(main())
