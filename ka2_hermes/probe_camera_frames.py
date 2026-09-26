#!/usr/bin/env python3
"""Grab one frame from the main road camera and one from the wide road camera, so the two can be
compared: field of view, resolution, and what detail the wide lens actually offers.

Read-only: attaches to the VisionIPC streams camerad already publishes. Nothing is sent to the car.

Frames are written as PGM (raw Y plane) - no image library needed on the box.
"""
import sys
import time

sys.path.insert(0, "/data/openpilot")
try:
  from msgq.visionipc import VisionIpcClient, VisionStreamType
except Exception as exc:                                   # pragma: no cover
  print("visionipc import failed: %s" % exc)
  raise SystemExit(1)

OUT = [("/tmp/main_road.pgm", "VISION_STREAM_ROAD"),
       ("/tmp/wide_road.pgm", "VISION_STREAM_WIDE_ROAD")]


def grab(stream_name, path, seconds=6.0):
  if not hasattr(VisionStreamType, stream_name):
    print("%-24s : this fork has no %s" % (stream_name, stream_name))
    return
  stream = getattr(VisionStreamType, stream_name)
  client = VisionIpcClient("camerad", stream, False)
  if not client.connect(False):
    print("%-24s : could not connect" % stream_name)
    return
  deadline = time.time() + seconds
  buf = None
  while time.time() < deadline:
    buf = client.recv()
    if buf is not None:
      break
  if buf is None:
    print("%-24s : connected but no frame arrived" % stream_name)
    return
  data = bytes(buf.data[:buf.width * buf.height])
  with open(path, "wb") as fh:
    fh.write(b"P5\n%d %d\n255\n" % (buf.width, buf.height))
    fh.write(data)
  print("%-24s : %dx%d stride %d -> %s (%d bytes of luma)"
        % (stream_name, buf.width, buf.height, buf.stride, path, len(data)))


for path, name in OUT:
  try:
    grab(name, path)
  except Exception as exc:
    print("%-24s : %s" % (name, exc))
