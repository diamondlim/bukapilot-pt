#!/usr/bin/env python3
"""Publish the modem's GNSS as a position - over AT, because port routing here is unreliable.

The EC25's GNSS engine runs, but its NMEA output port is a moving target: ModemManager reports ttyUSB1
as the gps port while the fork's qcomgpsd reads ttyUSB0, and both are held by something. The AT path
sidesteps that entirely - `AT+QGPSCFG="nmeasrc",1` makes sentences retrievable with AT+QGPSGNMEA, which
goes through the manager that already owns the ports: no stty, no port guessing, no fighting over a
character device.

It is written to be honest about a receiver with no signal. The heartbeat carries the satellite count and
how many of them report signal at all, so "engine running, antenna absent" is visible in the log instead
of looking like a broken bridge. The moment an antenna is fitted, fixes flow and positions land in
rlog/qlog with no change here.

Runs as the device user under the fork venv (cereal needs capnp); AT commands go through sudo.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ka2_gnss_pub import parse_sentence, state_to_json   # same NMEA logic as the serial bridge

GPS_JSON = os.environ.get("KA2_GPS_JSON", "/dev/shm/ka2_gps.json")
LOG_PATH = os.environ.get("KA2_GNSS_LOG", "/data/hermes/logs/ka2_gnss_at.log")
MMCLI = ["sudo", "-n", "mmcli", "-m", "0", "--command"]
PREFIX = re.compile(r"^\+QGPSGNMEA:\s*")


def log(msg):
    line = "%s  %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg)
    print(line, flush=True)
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        with open(LOG_PATH, "a") as handle:
            handle.write(line + "\n")
    except Exception:
        pass


def sentences_from_mmcli(raw):
    """Pull the NMEA out of an mmcli reply.

    mmcli wraps every reply as `response: '+QGPSGNMEA: $GPGGA,...'` and separates multiple sentences
    with a literal backslash-n, so neither the prefix nor the quoting can be assumed: find the dollar
    sign, take the rest of the line, and let the quotes fall away.
    """
    cleaned = raw.replace("\\r", "").replace("\\n", "\n")
    out = []
    for chunk in cleaned.split("\n"):
        start = chunk.find("$")
        if start < 0:
            continue
        sentence = chunk[start:].strip().strip("'").strip()
        if sentence.startswith("$"):
            out.append(sentence)
    return out


def ask(kind, timeout=12):
    """One AT command via the modem manager -> (sentences, error)."""
    try:
        done = subprocess.run(MMCLI + ["AT+QGPSGNMEA=\"%s\"" % kind],
                              capture_output=True, text=True, timeout=timeout)
    except Exception as exc:
        return [], "mmcli failed: %s" % exc
    raw = (done.stdout or "") + (done.stderr or "")
    found = sentences_from_mmcli(raw)
    if found:
        return found, None
    if "error" in raw.lower():
        code = re.search(r"error:?\s*(\d+)", raw)
        return [], "modem error %s" % (code.group(1) if code else raw.strip()[:70])
    return [], "no sentence in reply"


def parse_gsv(sentences):
    """(satellites in view, how many report any signal) - the second number is the antenna question."""
    total = with_signal = 0
    for sentence in sentences:
        parts = sentence.split(",")
        if len(parts) > 3 and parts[3].split("*")[0].isdigit():
            total = max(total, int(parts[3].split("*")[0]))
        for index in range(4, len(parts) - 3, 4):
            snr = parts[index + 3].split("*")[0].strip()
            if snr.isdigit() and int(snr) > 0:
                with_signal += 1
    return total, with_signal


def write_json(state, reason=None):
    try:
        payload = state_to_json(state) if state else {"ok": 0, "why": reason or "no fix"}
        if reason and payload.get("ok") == 0:
            payload["why"] = reason
        payload["source"] = "modem-at"
        payload["at"] = time.time()
        tmp = GPS_JSON + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(payload, handle)
        os.replace(tmp, GPS_JSON)
    except Exception as exc:
        log("could not write %s: %s" % (GPS_JSON, exc))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="poll and report, publish nothing")
    ap.add_argument("--gsv-every", type=float, default=15.0)
    ap.add_argument("--heartbeat-s", type=float, default=30.0)
    args = ap.parse_args()

    publish = None
    if not args.dry_run:
        import cereal.messaging as messaging
        from ka2_gnss_pub import publish as publish_position
        sock = messaging.pub_sock("gpsLocationExternal")
        publish = lambda st: publish_position(sock, st)   # noqa: E731

    state = {}
    polls = fixes = 0
    sats = sats_with_signal = 0
    last_gsv = last_beat = 0.0
    started = time.time()
    log("AT bridge up (%s), polling GGA every second" % ("dry run" if args.dry_run else "publishing"))

    while True:
        cycle = time.time()
        moved = False
        for sentence in ask("GGA")[0]:
            if parse_sentence(sentence, state):
                moved = True
        polls += 1

        if state.get("hasFix"):
            for sentence in ask("RMC")[0]:      # speed and course, only once there is a fix to hang them on
                parse_sentence(sentence, state)
            fixes += 1
            if publish is not None:
                publish(state)
            write_json(state)
            if fixes % 60 == 1:
                log("fix %d: %.6f, %.6f  %s sats, %.1f m/s" % (
                    fixes, state["latitude"], state["longitude"], state.get("sats"), state.get("speed") or 0.0))

        now = time.time()
        if now - last_gsv > args.gsv_every:
            last_gsv = now
            sats, sats_with_signal = parse_gsv(ask("GSV")[0])

        if now - last_beat > args.heartbeat_s:
            last_beat = now
            if sats and not sats_with_signal:
                rf = "no satellite reports signal (antenna/RF path); geometry from almanac only"
            elif sats:
                rf = "%d of %d satellites report signal" % (sats_with_signal, sats)
            else:
                rf = "no satellite data"
            log("heartbeat: polls %d, fixes %d, fix=%s, %s | up %.0f min" % (
                polls, fixes, "yes" if state.get("hasFix") else "no", rf, (now - started) / 60.0))
            if not state.get("hasFix"):
                write_json({}, reason=rf)   # so anything reading /dev/shm can say why, not just show nothing

        time.sleep(max(0.2, 1.0 - (time.time() - cycle)))


if __name__ == "__main__":
    sys.exit(main())
