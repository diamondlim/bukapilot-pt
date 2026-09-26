#!/usr/bin/env python3
"""Bring the KA2's own GPS module up and find where it talks.

Run on the box as root. The board carries a discrete GNSS module with its own power and reset lines
(`GET_GNSS_*`/`GPS_*` names in the vendor's own /usr/kommu board scripts), and the vendor software never
asserts them, so the chip is dark at boot and no serial port carries NMEA. Nothing is logged about it.

Two traps this script exists to avoid:

  * `export` only creates the control directory. A pin is drivable only after direction=out, and a value
    written while the pin is still an input is accepted and thrown away - it reads back unchanged and no
    error appears anywhere. Every write here is followed by a re-read of the same pin, and the *re-read*
    is what gets reported.
  * a tty read at the wrong baud returns silence, which is indistinguishable from a dead module, so each
    port is set to a speed with stty before it is read, at every baud worth trying.

The output is a verdict an engineer can act on: which lines are high, which port carries sentences, at
what speed, and which sentence families - or an explicit "no NMEA anywhere", which is a hardware answer
(no antenna, no sky view, or the wrong power line) rather than a software one.
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys
import time

VENDOR_GPIO_SH = "/usr/kommu/gpio.sh"
VENDOR_GLOB = "/usr/kommu/*.sh"
LOG_PATH = os.environ.get("KA2_GNSS_LOG", "/data/hermes/gnss_bringup.log")
DEBUG_GPIO = "/sys/kernel/debug/gpio"
SYSFS = "/sys/class/gpio"

# Only a fallback: the vendor's own board script is parsed first, because the pin numbers are a property
# of the board revision and guessing them is how you drive somebody else's line.
FALLBACK_PINS = {"GPS_PWR_EN": 34, "GPS_RST_N": 32, "GPS_SAFEBOOT_N": 33}
BAUDS = (9600, 115200, 38400, 460800)
PORTS = ["/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS5",
         "/dev/ttyS6", "/dev/ttyS7", "/dev/ttyS8", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
         "/dev/ttyACM0", "/dev/ttyACM1"]
SENTENCE_RE = re.compile(rb"\$G[A-Z]{3,5}")


def log(msg):
    line = "%s  %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg)
    print(line, flush=True)
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        with open(LOG_PATH, "a") as handle:
            handle.write(line + "\n")
    except Exception:
        pass


def sh(cmd, timeout=10):
    try:
        done = subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True, text=True, timeout=timeout)
        return done.returncode, done.stdout.strip(), done.stderr.strip()
    except Exception as exc:
        return 1, "", str(exc)


def vendor_pins():
    """Read the board's own script, so the pin numbers come from the vendor, not from us."""
    pins, source = {}, None
    candidates = [VENDOR_GPIO_SH] + sorted(glob.glob(VENDOR_GLOB))
    for path in candidates:
        if not os.path.exists(path):
            continue
        try:
            text = open(path, errors="replace").read()
        except Exception:
            continue
        # The vendor file lists "<pin>  # <NAME>" - the number comes FIRST. Pairing a name with the
        # next number found in the file (rather than the one on its own line) silently drives the
        # neighbouring line instead: on this board that meant pulsing LTE_BOOT while believing it was
        # GPS_RST_N. So pair within a line, in either order, and keep the line text for the log.
        found, lines_seen = {}, {}
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("#"):
                continue
            names = [n.rstrip("_") for n in re.findall(r"[A-Z][A-Z0-9_]{1,}", stripped)
                     if "GPS" in n or "GNSS" in n]
            numbers = [int(x) for x in re.findall(r"\b(\d{1,4})\b", stripped)]
            if not names or not numbers:
                continue
            for name in names:
                found.setdefault(name, numbers[0])
                lines_seen[name] = stripped
        if found:
            pins, source = found, path
            for name in sorted(pins):
                print("pin mapping: %s = %s   <- %r" % (name, pins[name], lines_seen.get(name)))
            break
    return pins, source


def read_line(name):
    try:
        with open(os.path.join(SYSFS, name, "value")) as handle:
            return int(handle.read().strip())
    except Exception:
        return None


def debug_gpio_claims():
    """Which driver, if any, already owns each interesting line - a claimed pin cannot be driven."""
    claims = {}
    try:
        text = open(DEBUG_GPIO, errors="replace").read()
    except Exception:
        return claims
    for line in text.splitlines():
        for name in ("GPS", "GNSS"):
            if name in line:
                claims[line.strip()] = line.strip()
    return claims


def pin_ready(num):
    """Export and set direction, then prove it: a pin still reading `in` cannot be driven."""
    path = os.path.join(SYSFS, "gpio%d" % num)
    if not os.path.isdir(path):
        try:
            with open(os.path.join(SYSFS, "export"), "w") as handle:
                handle.write(str(num))
            time.sleep(0.2)
        except Exception as exc:
            return None, "export failed: %s" % exc
    try:
        with open(os.path.join(path, "direction"), "w") as handle:
            handle.write("out")
    except Exception as exc:
        return None, "direction failed: %s" % exc
    try:
        direction = open(os.path.join(path, "direction")).read().strip()
    except Exception:
        direction = "?"
    if direction != "out":
        return None, "direction reads %r" % direction
    return path, None


def drive(num, value):
    """Write, then re-read. The re-read is the fact; the write's exit status is not."""
    path, err = pin_ready(num)
    if path is None:
        return None, err
    try:
        with open(os.path.join(path, "value"), "w") as handle:
            handle.write("1" if value else "0")
    except Exception as exc:
        return None, "write failed: %s" % exc
    time.sleep(0.05)
    return read_line("gpio%d" % num), None


def bring_up(pins, pulse_ms=250):
    """safeboot high (active-low: high is normal boot), reset pulse, then power - in that order.

    Resetting an unpowered module does nothing, which is why the order matters.
    """
    report = {}
    for name, num in sorted(pins.items()):
        if "SAFEBOOT" in name:
            report[name] = {"pin": num, "read_back": drive(num, 1)[0]}
    for name, num in sorted(pins.items()):
        if "RST" in name or "RESET" in name:
            drive(num, 0)
            time.sleep(pulse_ms / 1000.0)
            report.setdefault(name, {})["pin"] = num
            report[name]["read_back_after_pulse"] = drive(num, 1)[0]
    for name, num in sorted(pins.items()):
        if "PWR" in name or "EN" in name:
            report.setdefault(name, {})["pin"] = num
            report[name]["read_back"] = drive(num, 1)[0]
    return report


def sweep(ports, bauds, dwell_s=3.0):
    """Set a speed on each port before reading it, then look for sentence starts only."""
    found = []
    for port in ports:
        if not os.path.exists(port):
            continue
        for baud in bauds:
            rc, _, err = sh("stty -F %s %d raw -echo -crtscts" % (port, baud))
            if rc != 0:
                found.append({"port": port, "baud": baud, "error": err[:80]})
                continue
            rc, out, _ = sh("timeout %d cat %s" % (int(dwell_s), port), timeout=dwell_s + 3)
            hits = SENTENCE_RE.findall(out.encode("utf-8", "replace"))
            if hits:
                families = sorted(set(h.decode() for h in hits))
                found.append({"port": port, "baud": baud, "sentences": len(hits), "families": families})
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep-only", action="store_true", help="do not touch any pin, just look for NMEA")
    ap.add_argument("--port", action="append", default=None, help="limit the sweep (repeatable)")
    ap.add_argument("--baud", action="append", type=int, default=None)
    ap.add_argument("--json", default="/dev/shm/ka2_gnss_find.json", help="where to leave the verdict")
    args = ap.parse_args()

    pins, source = vendor_pins()
    if not pins:
        pins, source = FALLBACK_PINS, "fallback (vendor script unreadable)"
    log("board script: %s" % source)
    log("pins as read: %s" % json.dumps(pins, sort_keys=True))

    claims = debug_gpio_claims()
    for line in claims:
        log("already claimed: %s" % line)

    result = {"pins": pins, "source": source, "claims": list(claims), "pin_states": {}, "nmea": []}
    if not args.sweep_only:
        result["pin_states"] = bring_up(pins)
        for name, state in sorted(result["pin_states"].items()):
            log("pin %s -> %s" % (name, json.dumps(state, sort_keys=True)))
        time.sleep(2.0)          # the chip needs a moment before it answers on its port

    result["nmea"] = sweep(args.port or PORTS, args.baud or list(BAUDS))
    if result["nmea"]:
        for hit in result["nmea"]:
            if "families" in hit:
                log("NMEA on %s at %d: %d sentences, %s" % (hit["port"], hit["baud"], hit["sentences"],
                                                            ",".join(hit["families"])))
    else:
        log("no NMEA on any port at any speed - that is a hardware answer (antenna, sky view, or the "
            "power line is not the one the vendor script names), not a software one")

    verdict = {"pins": result["pins"], "pin_states": result["pin_states"],
               "nmea": [h for h in result["nmea"] if "families" in h], "at": time.time()}
    try:
        tmp = args.json + ".tmp"
        with open(tmp, "w") as handle:
            json.dump(verdict, handle)
        os.replace(tmp, args.json)
    except Exception as exc:
        log("could not write %s: %s" % (args.json, exc))
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
