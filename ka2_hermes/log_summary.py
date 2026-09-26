#!/usr/bin/env python3
"""Drive summaries, for the app's log page to read.

The app cannot compute these - reading a drive's logs takes seconds, and the Bluetooth link is
measured in kilobytes per second. So they are computed here, once per drive, and cached; the app
only ever reads the result.

    list                the recent drives that actually moved, newest first
    summary <route>     the numbers for one drive (cached after the first time)

Only drives that moved are listed: the box logs continuously whenever it is powered, so most of
what is on disk is a parked car with the ignition on. The bar is 5 km/h - a parked car reads 0, and
anything actually driving clears it, slow town work included.
"""
import glob
import json
import math
import os
import statistics
import sys
import time

sys.path.insert(0, "/data/openpilot")
from openpilot.tools.lib.logreader import LogReader

REALDATA = "/data/media/0/realdata"
CACHE = "/dev/shm/ka2_log_summaries.json"
LIST_CACHE = "/dev/shm/ka2_list_cache.json"
LOOKUPS = (5.0, 10.0)
MIN_MEDIAN_SPEED_MS = 5.0 / 3.6   # 5 km/h: below this the car was parked or creeping
MAX_SEGMENTS_READ = 3         # per drive, so a summary stays quick


def _speeds(path):
    try:
        return [float(m.carState.vEgo) for m in LogReader(path) if m.which() == "carState"]
    except Exception:
        return []


def _segments(route_dir):
    return sorted(glob.glob(os.path.join(route_dir, "rlog.zst")))


def _entry_from_speeds(name, seg_count, sp):
    started = name.split("--")[0] + " " + name.split("--")[1].replace("-", ":")
    if not sp or statistics.median(sp) <= MIN_MEDIAN_SPEED_MS:
        return {"skip": True}
    return {
        "route": name,
        "started": started,
        "frames": len(sp) * max(1, seg_count),
        "median_speed_ms": round(statistics.median(sp), 2),
        "max_speed_ms": round(max(sp), 2),
    }


def list_drives(limit=8, max_scan=24):
    """Recent drives that moved, newest first.

    Reading a route's rlog is the expensive part (a multi-megabyte decompress per route, ~15 s for a
    list of eight), so each route's entry is cached under a key that carries the last segment's size
    and mtime: a finished drive is computed once ever, and only the drive still being written is
    re-read on each call. Without this, every tap of the app's drives page cost the whole list again.
    """
    try:
        with open(LIST_CACHE) as fh:
            store = json.load(fh)
    except Exception:
        store = {}
    dirty = False
    out = []
    for d in sorted(glob.glob(os.path.join(REALDATA, "2026-*")), reverse=True)[:max_scan]:
        segs = _segments(d)
        if not segs:
            continue
        name = os.path.basename(d)
        try:
            st = os.stat(segs[-1])
            key = "%s|%d|%d" % (name, st.st_size, int(st.st_mtime))
        except OSError:
            key = name
        entry = store.get(key)
        if entry is None:
            entry = _entry_from_speeds(name, len(segs), _speeds(segs[-1]))
            store[key] = entry
            dirty = True
        if entry.get("skip"):
            continue
        out.append(entry)
        if len(out) >= limit:
            break
    if dirty:
        try:
            keep = dict(list(store.items())[-400:])     # bounded: this file lives in RAM
            os.makedirs(os.path.dirname(LIST_CACHE), exist_ok=True)
            with open(LIST_CACHE, "w") as fh:
                json.dump(keep, fh)
        except Exception:
            pass
    return out


def _at(items, target):
    xs = [float(v) for v in items.x]
    ys = [float(v) for v in items.y]
    for i in range(len(xs) - 1):
        if xs[i] <= target <= xs[i + 1] and xs[i + 1] > xs[i]:
            f = (target - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + f * (ys[i + 1] - ys[i])
    return None


def summarise(route):
    """The numbers for one drive: centring, lane availability, speed."""
    route_dir = route if route.startswith("/") else os.path.join(REALDATA, route)
    segs = _segments(route_dir)[:MAX_SEGMENTS_READ]
    if not segs:
        return {"error": "no logs for %s" % route}

    offsets = {d: [] for d in LOOKUPS}
    speeds, probs_l, probs_r = [], [], []
    frames = 0
    for seg in segs:
        try:
            for m in LogReader(seg):
                w = m.which()
                if w == "carState":
                    speeds.append(float(m.carState.vEgo))
                elif w == "modelV2":
                    frames += 1
                    pl = [float(p) for p in m.modelV2.laneLineProbs]
                    probs_l.append(pl[1])
                    probs_r.append(pl[2])
                    if len(m.modelV2.laneLines) > 2:
                        left, right = m.modelV2.laneLines[1], m.modelV2.laneLines[2]
                        for d in LOOKUPS:
                            a, b = _at(left, d), _at(right, d)
                            if a is not None and b is not None:
                                offsets[d].append((a + b) / 2.0)
        except Exception as exc:
            return {"error": "could not read %s: %s" % (os.path.basename(seg), exc)}

    if not frames or not speeds:
        return {"error": "no model frames in %s" % route}

    faint = sum(1 for a, b in zip(probs_l, probs_r) if min(a, b) < 0.5)
    out = {
        "route": os.path.basename(route_dir),
        "frames": frames,
        "duration_s": round(len(speeds) / 100.0, 1),
        "median_speed_ms": round(statistics.median(speeds), 2),
        "max_speed_ms": round(max(speeds), 2),
        "lane_below_threshold_pct": round(100.0 * faint / frames, 1),
        "lane_confidence": [round(statistics.median(probs_l), 2), round(statistics.median(probs_r), 2)],
        "centring": {},
    }
    for d in LOOKUPS:
        s = offsets[d]
        if not s:
            out["centring"]["%dm" % d] = None
            continue
        out["centring"]["%dm" % d] = {
            "median_m": round(statistics.median(s), 3),
            "rms_m": round(math.sqrt(sum(x * x for x in s) / len(s)), 3),
        }
    return out


def cached_summary(route):
    try:
        with open(CACHE) as fh:
            store = json.load(fh)
    except Exception:
        store = {}
    if route in store:
        return store[route]
    s = summarise(route)
    s["computed_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    store[route] = s
    os.makedirs(os.path.dirname(CACHE), exist_ok=True)
    with open(CACHE, "w") as fh:
        json.dump(store, fh, indent=1)
    return s


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "list"
    if what == "list":
        print(json.dumps(list_drives(), indent=1))
    elif what == "summary":
        print(json.dumps(cached_summary(sys.argv[2]), indent=1))
    else:
        print("usage: log_summary.py list | summary <route>")
