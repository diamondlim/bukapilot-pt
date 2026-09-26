#!/usr/bin/env python3
"""Publish the KA2's own GPS as the position message the rest of the system logs.

The board's GNSS module talks NMEA on a serial port; nothing in this fork reads that module, and this
fork's locationd never consumes a position at all (checked: no `gps` reference in locationd.py), so
nothing on the device turns NMEA into a position. This bridge does, and publishing it means the fix
lands in rlog/qlog - which is what makes a drive plottable on a map afterwards.

Must run under the fork venv (`/usr/local/venv/bin/python3`): `cereal` needs capnp, which the system
python (the one with dbus/gi that the Bluetooth service uses) does not have. That is the same split the
pose publisher lives with.

Usage
    /usr/local/venv/bin/python3 ka2_gnss_pub.py --port /dev/ttyS3 --baud 9600            # publish
    /usr/local/venv/bin/python3 ka2_gnss_pub.py --port /dev/ttyS3 --baud 9600 --dry-run  # just parse
"""
import argparse
import json
import os
import sys
import time

GPS_JSON = os.environ.get("KA2_GPS_JSON", "/dev/shm/ka2_gps.json")
LOG_PATH = os.environ.get("KA2_GNSS_LOG", "/data/hermes/logs/ka2_gnss_pub.log")
# HDOP is the only accuracy figure NMEA carries, and it is not metres. 5 m per HDOP unit is the
# conventional first-order mapping; it is an estimate and is labelled as one rather than dressed up.
METRES_PER_HDOP = 5.0
STALE_S = 5.0            # no sentence for this long and the fix is published as lost


def log(msg):
    line = "%s  %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg)
    print(line, flush=True)
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        with open(LOG_PATH, "a") as handle:
            handle.write(line + "\n")
    except Exception:
        pass


def checksum_ok(sentence):
    """NMEA checksum: XOR of everything between $ and *."""
    body = sentence[1:]
    if "*" not in body:
        return False
    payload, _, given = body.partition("*")
    total = 0
    for char in payload:
        total ^= ord(char)
    try:
        return total == int(given[:2], 16)
    except ValueError:
        return False


def _dm_to_deg(value, hemisphere):
    """NMEA gives ddmm.mmmm / dddmm.mmmm - degrees and decimal minutes, not decimal degrees."""
    if not value:
        return None
    try:
        dot = value.index(".")
    except ValueError:
        return None
    degrees = float(value[:dot - 2]) if dot >= 2 else 0.0
    minutes = float(value[dot - 2:])
    out = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        out = -out
    return out


def _nmea_epoch_ms(date_field, time_field):
    """UTC from the sentence itself: the fix carries satellite time, so a wrong box clock cannot
    silently misdate a track."""
    if not date_field or not time_field:
        return None
    try:
        day, month, year = int(date_field[0:2]), int(date_field[2:4]), int(date_field[4:6])
        year += 2000 if year < 80 else 1900
        hour = int(time_field[0:2])
        minute = int(time_field[2:4])
        second = float(time_field[4:])
        import calendar
        stamp = calendar.timegm((year, month, day, hour, minute, int(second), 0, 0, 0))
        return int((stamp + (second - int(second))) * 1000)
    except Exception:
        return None


def parse_sentence(line, state):
    """Update `state` from one NMEA sentence. Returns True when the position itself changed.

    GGA carries the fix quality and altitude, RMC the position and course/speed over ground; either can
    arrive first, so both write into the same dict and the position is published when one of them moves.
    """
    if not line.startswith("$"):
        return False
    fields = line.strip().split(",")
    kind = fields[0][-3:]
    moved = False

    if kind == "GGA" and len(fields) >= 10:
        quality = fields[6] or "0"
        state["sats"] = int(fields[7]) if fields[7].isdigit() else None
        state["hdop"] = float(fields[8]) if fields[8] else None
        state["altitude"] = float(fields[9]) if fields[9] else None
        state["hasFix"] = quality not in ("", "0")
        lat, lon = _dm_to_deg(fields[2], fields[3]), _dm_to_deg(fields[4], fields[5])
        if lat is not None and lon is not None and state["hasFix"]:
            if (lat, lon) != (state.get("latitude"), state.get("longitude")):
                state["latitude"], state["longitude"], moved = lat, lon, True
        state["stamp_ms"] = _nmea_epoch_ms(state.get("date"), fields[1]) or state.get("stamp_ms")
        state["kind"] = "GGA"

    elif kind == "RMC" and len(fields) >= 8:
        state["date"] = fields[9] if len(fields) > 9 else state.get("date")
        state["hasFix"] = fields[2] == "A"
        lat, lon = _dm_to_deg(fields[3], fields[4]), _dm_to_deg(fields[5], fields[6])
        if lat is not None and lon is not None and state["hasFix"]:
            if (lat, lon) != (state.get("latitude"), state.get("longitude")):
                state["latitude"], state["longitude"], moved = lat, lon, True
        state["speed"] = float(fields[7]) * 0.514444 if fields[7] else None      # knots -> m/s
        state["bearing"] = float(fields[8]) if len(fields) > 8 and fields[8] else None
        state["stamp_ms"] = _nmea_epoch_ms(state["date"], fields[1]) or state.get("stamp_ms")
        state["kind"] = "RMC"

    elif kind == "GNS" and len(fields) >= 8:
        # GNS aggregates constellations, so its mode field is a *string* of per-constellation letters
        # ("AAA" = GPS+GLONASS+Galileo autonomous, "DDD" = all three differential), not one status
        # letter. And a sentence with an empty position is only reporting satellites for one
        # constellation - it says nothing about the fix, so it must not clear one.
        mode, has_position = fields[6], bool(fields[2] and fields[4])
        if has_position:
            state["hasFix"] = any(char in "ADPRFEM" for char in mode)
            lat, lon = _dm_to_deg(fields[2], fields[3]), _dm_to_deg(fields[4], fields[5])
            if lat is not None and lon is not None and state["hasFix"]:
                if (lat, lon) != (state.get("latitude"), state.get("longitude")):
                    state["latitude"], state["longitude"], moved = lat, lon, True
            if fields[7].isdigit():
                state["sats"] = int(fields[7])          # aggregate count, valid with a position
            if len(fields) > 8 and fields[8]:
                state["hdop"] = float(fields[8])
            if len(fields) > 9 and fields[9]:
                state["altitude"] = float(fields[9])
            state["stamp_ms"] = _nmea_epoch_ms(state.get("date"), fields[1]) or state.get("stamp_ms")

    elif kind == "VTG" and len(fields) >= 8:
        state["bearing"] = float(fields[1]) if fields[1] else state.get("bearing")
        if fields[7]:
            state["speed"] = float(fields[7]) / 3.6                              # km/h -> m/s
        state["kind"] = "VTG"

    return moved


def state_to_json(state):
    if not state.get("hasFix"):
        return {"ok": 0, "why": "no fix"}
    return {"ok": 1, "lat": round(state["latitude"], 7), "lon": round(state["longitude"], 7),
            "alt": state.get("altitude"), "speed_ms": state.get("speed"),
            "bearing": state.get("bearing"), "sats": state.get("sats"),
            "hdop": state.get("hdop"), "stamp_ms": state.get("stamp_ms"),
            "at": time.time()}


def write_json(state):
    try:
        tmp = GPS_JSON + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(state_to_json(state), handle)
        os.replace(tmp, GPS_JSON)
    except Exception as exc:
        log("could not write %s: %s" % (GPS_JSON, exc))


def publish(sock, state):
    import cereal.messaging as messaging
    msg = messaging.new_message("gpsLocationExternal", valid=bool(state.get("hasFix")))
    gps = msg.gpsLocationExternal
    gps.flags = 1 if state.get("hasFix") else 0
    gps.hasFix = bool(state.get("hasFix"))
    if state.get("latitude") is not None:
        gps.latitude = float(state["latitude"])
        gps.longitude = float(state["longitude"])
    gps.altitude = float(state.get("altitude") or 0.0)
    gps.speed = float(state.get("speed") or 0.0)
    gps.bearingDeg = float(state.get("bearing") or 0.0)
    hdop = state.get("hdop") or 9.99
    gps.horizontalAccuracy = float(hdop) * METRES_PER_HDOP
    gps.verticalAccuracy = float(hdop) * METRES_PER_HDOP * 1.5
    gps.speedAccuracy = 0.5
    gps.bearingAccuracyDeg = 5.0
    gps.satelliteCount = int(state.get("sats") or 0)
    gps.source = "external"                      # SensorSource.external: a module we read ourselves
    gps.unixTimestampMillis = int(state.get("stamp_ms") or time.time() * 1000)
    if state.get("speed") is not None and state.get("bearing") is not None:
        import math
        rad = math.radians(state["bearing"])
        gps.vNED = [state["speed"] * math.cos(rad), state["speed"] * math.sin(rad), 0.0]
    sock.send(msg.to_bytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--dry-run", action="store_true", help="parse and report, do not publish")
    ap.add_argument("--stale-s", type=float, default=STALE_S)
    args = ap.parse_args()

    state = {}
    sock = None
    if not args.dry_run:
        import cereal.messaging as messaging
        sock = messaging.pub_sock("gpsLocationExternal")

    log("reading %s at %d baud (%s)" % (args.port, args.baud, "dry run" if args.dry_run else "publishing"))
    import subprocess
    subprocess.run(["stty", "-F", args.port, str(args.baud), "raw", "-echo", "-crtscts"],
                   capture_output=True, check=False)

    sentences = bad = fixes = 0
    last_fix = last_pub = 0.0
    while True:
        try:
            handle = open(args.port, "rb", buffering=0)
        except Exception as exc:
            log("cannot open %s: %s (retrying)" % (args.port, exc))
            time.sleep(3)
            continue
        try:
            for raw in handle:
                line = raw.decode("ascii", "replace").strip()
                if not line.startswith("$"):
                    continue
                sentences += 1
                if not checksum_ok(line):
                    bad += 1
                    continue
                if parse_sentence(line, state) or (state.get("hasFix") and time.time() - last_pub > 1.0):
                    last_pub = time.time()
                    if state.get("hasFix"):
                        fixes += 1
                        last_fix = time.time()
                        if sock is not None:
                            publish(sock, state)
                        write_json(state)
                        if fixes % 60 == 1:
                            log("fix %d: %.6f, %.6f  %s sats, hdop %s, %.1f m/s" % (
                                fixes, state["latitude"], state["longitude"], state.get("sats"),
                                state.get("hdop"), state.get("speed") or 0.0))
                    else:
                        write_json(state)
                if sock is not None and state.get("hasFix") and time.time() - last_fix > args.stale_s:
                    state["hasFix"] = False          # say the fix was lost rather than repeating a stale one
                    publish(sock, state)
                    write_json(state)
                    log("fix lost (no sentence for %.0fs)" % args.stale_s)
        except Exception as exc:
            log("read error: %s (reopening)" % exc)
            time.sleep(2)
        finally:
            try:
                handle.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
