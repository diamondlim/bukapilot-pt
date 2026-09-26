#!/usr/bin/env python3
"""Read the car's live CAN traffic and look for the accelerate/brake interface.

Read-only: it subscribes to the `can` stream that card already receives. Nothing is transmitted.

What it reports:
  * every message id on the bus, with its name from the DBC the port uses, and its rate
  * the ACC messages specifically, with ACC_CMD decoded: ACCEL_CMD is byte 0 minus 100, and the car
    itself states ACC_CONTROLLABLE_AND_ON
  * whether the fork's other longitudinal messages (ACC_MPC_STATE, ACC_EPS_STATE) exist on this car
    at all - the mpc_lka path needs them and this car is a cam_lka car
"""
import collections
import os
import re
import sys
import time

sys.path.insert(0, "/data/openpilot")
import cereal.messaging as messaging  # noqa: E402

DBC = "/data/openpilot/opendbc_repo/opendbc/dbc/byd_general_pt.dbc"
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
WATCH = {"ACC_CMD", "ACC_HUD_ADAS", "ACC_MPC_STATE", "ACC_EPS_STATE", "LKAS_HUD_ADAS",
         "PCM_BUTTONS", "STEERING_MODULE_ADAS", "ADAS2", "ADAS4", "ADAS6"}


def names_from_dbc(path):
  """address -> message name, straight from the DBC the port uses."""
  out = {}
  try:
    with open(path, "r", errors="replace") as fh:
      for line in fh:
        m = re.match(r"BO_ (\d+) (\w+):", line)
        if m:
          out[int(m.group(1))] = m.group(2)
  except Exception as exc:
    print("could not read %s: %s" % (path, exc))
  return out


def bits_lsb(data, start, length):
  """DBC '@1' (little endian) signal starting at a DBC bit number."""
  value = 0
  for i in range(length):
    bit = start + i
    if bit // 8 >= len(data):
      return None
    value |= ((data[bit // 8] >> (bit % 8)) & 1) << i
  return value


def bits_motorola(data, start, length):
  """DBC '@0' (big endian) signal: bit numbers count from the LSB of each byte."""
  value = 0
  bit = start
  for _ in range(length):
    byte, off = bit // 8, bit % 8
    if byte >= len(data):
      return None
    value = (value << 1) | ((data[byte] >> off) & 1)
    bit = bit - 1 if off > 0 else bit + 15
  return value


def main():
  names = names_from_dbc(DBC)
  print("DBC: %s (%d messages known)" % (DBC, len(names)))
  print("reading the bus for %.0f s - read only, nothing is transmitted" % SECONDS)
  print()

  sm = messaging.SubMaster(["can", "sendcan", "carState"])
  counts = collections.Counter()
  frames = {}
  accel_seen = collections.Counter()
  acc_flags = collections.Counter()
  first_shape = None
  t0 = time.time()
  while time.time() - t0 < SECONDS:
    sm.update(100)
    for service in ("can", "sendcan"):
      if not sm.updated[service]:
        continue
      payload = sm[service]
      # SubMaster hands back the list itself for these; older builds hand back the event
      for m in getattr(payload, service, payload):
        if first_shape is None:
          first_shape = sorted(m.to_dict().keys())
        key = (int(m.address), getattr(m, "src", -1),
               "TX" if service == "sendcan" else "RX")
        counts[key] += 1
        if names.get(int(m.address)) in WATCH or (service == "sendcan"):
          frames.setdefault(key, m.dat)
        if int(m.address) == 814:                       # ACC_CMD
          data = bytes(m.dat)
          if len(data) >= 8:
            accel_seen[data[0] - 100] += 1
            acc_flags[(bits_motorola(data, 44, 1), bits_motorola(data, 9, 1))] += 1

  print("fields on a can entry: %s" % first_shape)
  print("%d distinct message streams seen" % len(counts))
  print()
  print("%-6s %-4s %-5s %-24s %8s" % ("id", "bus", "dir", "name", "rate"))
  print("-" * 56)
  for (address, src, direction), n in sorted(counts.items(), key=lambda kv: -kv[1])[:40]:
    print("%-6d %-4d %-5s %-24s %6.1f Hz" % (
        address, src, direction, names.get(address, "?"), n / SECONDS))

  print()
  watched = [(k, n) for k, n in counts.items() if names.get(k[0]) in WATCH]
  print("=== the messages that matter for accelerate/brake ===")
  if not watched:
    print("none of %s are on this car's bus" % sorted(WATCH))
  for (address, src, direction), n in sorted(watched):
    print("  %-24s id %-5d bus %-3d %-3s %6.1f Hz" % (
        names.get(address), address, src, direction, n / SECONDS))

  print()
  print("=== what ACC_CMD (0x32E) was saying ===")
  if not accel_seen:
    print("  ACC_CMD was not seen on the bus")
  else:
    print("  ACCEL_CMD values (raw byte - 100): %s" % dict(accel_seen.most_common(8)))
    print("  (ACC_CONTROLLABLE_AND_ON, ACC_ON_1): %s" % dict(acc_flags.most_common(4)))
    print("  frames carrying it: %d" % sum(accel_seen.values()))

  if sm.updated["carState"]:
    cs = sm["carState"]
    print()
    print("carState alongside it: vEgo %.2f m/s, cruise %.2f m/s, available %s, enabled %s"
          % (cs.vEgo, cs.cruiseState.speed, cs.cruiseState.available, cs.cruiseState.enabled))


if __name__ == "__main__":
  main()
