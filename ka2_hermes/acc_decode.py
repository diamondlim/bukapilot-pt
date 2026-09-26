#!/usr/bin/env python3
"""Decoding the car's own acceleration command, straight off the bus.

`ACC_CMD` (0x32E) is the message the car's ADAS module sends 50 times a second to ask for acceleration
or braking. Reading it tells the driver what the car is doing on its own, and it is the same message
openpilot's longitudinal path would take over - which this platform does not enable.

Signal layout comes from the DBC the port uses (opendbc/dbc/byd_general_pt.dbc):

  ACCEL_CMD                    0|8@1+   (1,-100)     the request itself
  ACC_ON_1                     9|1@0+
  ACC_ON_2                    17|1@0+
  ACC_CONTROLLABLE_AND_ON     44|1@0+                the car saying its ACC may be commanded
  ACC_OVERRIDE_OR_STANDSTILL  45|1@0+
  STANDSTILL_STATE            40|1@0+
  SET_ME_25_1                 10|6@1+                always 25
  SET_ME_25_2                 18|6@1+

SET_ME_25_1 and _2 being 25 in every frame is what makes this decoder checkable against the real bus:
they are constants, so a wrong bit order shows up immediately instead of silently producing numbers.
"""
ACC_CMD_ADDRESS = 814

DBC = "/data/openpilot/opendbc_repo/opendbc/dbc/byd_general_pt.dbc"


def bits_lsb(data, start, length):
  """DBC '@1' (little endian): bits count up from the given bit number."""
  value = 0
  for i in range(length):
    bit = start + i
    if bit // 8 >= len(data):
      return None
    value |= ((data[bit // 8] >> (bit % 8)) & 1) << i
  return value


def bits_motorola(data, start, length):
  """DBC '@0' (big endian): bit numbers count from the LSB of each byte, moving to lower bits first."""
  value = 0
  bit = start
  for _ in range(length):
    byte, off = bit // 8, bit % 8
    if byte >= len(data):
      return None
    value = (value << 1) | ((data[byte] >> off) & 1)
    bit = bit - 1 if off > 0 else bit + 15
  return value


def decode_acc_cmd(data):
  """One ACC_CMD frame -> the car's acceleration request. None if the frame is not usable."""
  if data is None or len(data) < 8:
    return None
  raw = bits_lsb(data, 0, 8)
  if raw is None:
    return None
  return {"cmd": raw - 100,                      # ACCEL_CMD: scale 1, offset -100
          "on": int(bits_motorola(data, 9, 1) or 0),
          "on2": int(bits_motorola(data, 17, 1) or 0),
          "ctrl": int(bits_motorola(data, 44, 1) or 0),
          "ovr": int(bits_motorola(data, 45, 1) or 0),
          "still": int(bits_motorola(data, 40, 1) or 0)}


def frame_is_sane(data):
  """The two SET_ME_25 constants must both read 25; if not, the frame is not what we think it is."""
  if data is None or len(data) < 8:
    return False
  if bits_lsb(data, 10, 6) != 25 or bits_lsb(data, 18, 6) != 25:
    return False
  return True
