#!/usr/bin/env python3
"""Press the stock ACC's buttons from the box (runs ON the KA2, not on the host).

Why this exists: the car's ACC set speed is a driver-intent button message (0x3B0 PCM_BUTTONS), not
something the ADAS bus can request. The fork's safety mode already allows transmitting that message
(its TX hook gates steering angle and ACC_CMD, and says nothing about buttons; the allow-list has
0x3B0 on bus 0 and bus 2), so no firmware change is needed to nudge an ACC the driver already set.

**This tool never transmits anything itself.** It writes a request file that card consumes and sends from
inside card, because msgq allows exactly one publisher per topic and `sendcan` is card's: publishing it
from here made msgq raise MultiplePublishersError inside card and killed the car daemon (24 Sep 2026).

Gates kept deliberately strict, because this moves a car's longitudinal setpoint:
  * LKAS_ON_BTN is never sent - that is the driver's lane-keep switch, not ours.
  * ACC must be **engaged** (carState.cruiseState.enabled), not merely switched on: a press is refused
    otherwise, so this cannot act on a car whose ACC is idle or off. card re-checks the same thing.
  * Minimum gap between presses, one press per request, and every press is appended to an audit log.
  * card re-checks all of this, and also ignores a request older than 3 s rather than firing it late.

Packing note: the constants SET_ME_1_1 (byte0 bit2) and SET_ME_1_2 (byte1 bit4) must be 1. That was
verified against a real transmit captured from this car's own logs - a cancel press the box actually
sent was 04 10 08 00 00 00 00 00, i.e. byte0=0x04 (constant), byte1=0x10 (constant), byte2=0x08 (cancel).

Interpreter: the service that calls this runs on /usr/bin/python3, which has no capnp/cereal. Run it
with /usr/local/venv/bin/python3 (what the box's other publisher scripts use); the path bootstrap below
also makes plain `python3` work when /data/openpilot is importable.

Use (on the box):
  /usr/local/venv/bin/python3 /data/hermes/ka2_acc_press.py --button res      # one press
  /usr/local/venv/bin/python3 /data/hermes/ka2_acc_press.py --state           # what the car reports
  /usr/local/venv/bin/python3 /data/hermes/ka2_acc_press.py --show-frames     # packing, no car needed
"""
import argparse
import json
import os
import sys
import time

BUTTONS = {
    # name: bits to set -- from opendbc/dbc/byd_general_pt.dbc message 944 PCM_BUTTONS, corrected against
    # the car's own transmits (which is how we know the speed rocker sets SET and RES together)
    "step": ((0, 3), (0, 4)),   # the car's own ACC speed-button pattern
    "set": ((0, 3),),           # SET_BTN            - typically decrease / set
    "res": ((0, 4),),           # RES_BTN            - typically increase / resume
    "lkas": ((0, 6),),          # LKAS_ON_BTN        - never sent by this tool
    "dec_dist": ((1, 7),),      # DEC_DISTANCE_BTN
    "inc_dist": ((2, 0),),      # INC_DISTANCE_BTN
    "cancel": ((2, 3),),        # ACC_ON_BTN         - ACC off/cancel
}
CONSTANTS = {(0, 2): 1, (1, 4): 1}     # SET_ME_1_1, SET_ME_1_2 (verified against a real transmit)
ADDR_PCM_BUTTONS = 0x3B0
BUTTON_BUS = 0                          # Sealion 7 uses main bus; Atto-style platforms use cam bus (2)
MIN_GAP_S = 0.35                        # refuse presses closer together than this
AUDIT = "/data/hermes/acc_presses.jsonl"
ACC_REQUEST = "/data/hermes/acc/request"   # card reads, consumes and deletes this file
_last_press = [0.0]                     # per-process guard; the service is one long-lived process

# openpilot's python modules live in the repo, not in the venv (cereal, capnp, opendbc)
for _p in ("/data/openpilot", "/data/openpilot/opendbc_repo"):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)


def pack(button=None):
    """Return the 8 data bytes for a PCM_BUTTONS frame with at most one button pressed."""
    data = bytearray(8)
    for (byte, bit), val in CONSTANTS.items():
        if val:
            data[byte] |= 1 << bit
    if button is not None:
        if button not in BUTTONS:
            raise ValueError("unknown button %r (have: %s)" % (button, ", ".join(BUTTONS)))
        if button == "lkas":
            raise ValueError("refusing to press LKAS_ON_BTN - that is the driver's lane-keep switch")
        for byte, bit in BUTTONS[button]:
            data[byte] |= 1 << bit
    return bytes(data)


def audit(entry):
    entry["t"] = time.time()
    try:
        os.makedirs(os.path.dirname(AUDIT), exist_ok=True)
        with open(AUDIT, "a") as fh:
            fh.write(json.dumps(entry) + "\n")
    except Exception:
        pass          # never let logging block a press, or the reason for the press is lost


def _kph(value):
    return "%.0f" % value if isinstance(value, float) else "-"


def acc_state(timeout_s=1.0):
    """Current ACC state from the car, via cereal. Returns (available, enabled, set_speed_kph|error)."""
    try:
        from cereal import messaging
        sm = messaging.SubMaster(["carState"])
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            sm.update(100)
            if sm.updated["carState"]:
                cs = sm["carState"].cruiseState
                return bool(cs.available), bool(cs.enabled), float(cs.speedCluster) * 3.6
    except Exception as exc:
        return None, None, "submaster failed: %s" % str(exc)[:80]
    return None, None, "no carState in %.1fs" % timeout_s


def state_line():
    """One line describing what the car says about ACC right now."""
    avail, enabled, extra = acc_state()
    if avail is None:
        return False, "state unreadable (%s)" % extra
    return True, "state: ACC available=%s enabled=%s set=%s kph" % (avail, enabled, _kph(extra))


def press(button, dry_run=False, settle_s=1.2, frames=None):
    """Write the request card will pick up, then read back what the car reports. Returns (ok, sentence)."""
    if button == "lkas":
        return False, "refused: LKAS_ON is the driver's switch"

    # A hand cannot press a button 3 times a second, and neither should software. This is the only
    # thing standing between a stuck UI button and a stream of presses at the car.
    now = time.time()
    if now - _last_press[0] < MIN_GAP_S:
        return False, "refused: %.0f ms since the last press (minimum %d ms)" % (
            (now - _last_press[0]) * 1000.0, int(MIN_GAP_S * 1000))

    avail, enabled, extra = acc_state()
    if avail is None:
        return False, "refused: could not read carState (%s)" % extra
    if not avail:
        return False, "refused: ACC is not switched on in the car (available=%s enabled=%s)" % (avail, enabled)
    if not enabled:
        # Deliberate: the owner wants presses only while ACC is actually active and driving, not merely
        # switched on. Switched-on-but-idle and engaged behave differently on this car anyway - the "+"
        # pattern is resume/cancel at a standstill.
        return False, "refused: ACC is switched on but not engaged (set=%s kph)" % _kph(extra)

    if dry_run:
        return True, "dry run: would request %s (%s) | ACC available=%s enabled=%s set=%s kph" % (
            button, pack(button).hex(), avail, enabled, _kph(extra))

    try:
        os.makedirs(os.path.dirname(ACC_REQUEST), exist_ok=True)
        tmp = "%s.tmp.%d" % (ACC_REQUEST, os.getpid())
        with open(tmp, "w") as fh:
            # "<button> <unix_ms> [frames]" - card clamps the frame count
            fh.write("%s %d%s" % (button, int(time.time() * 1000), "" if frames is None else " %d" % frames))
        os.replace(tmp, ACC_REQUEST)       # atomic: card never reads a half-written request
    except OSError as exc:
        return False, "refused: could not write the request (%s)" % str(exc)[:80]

    _last_press[0] = time.time()
    audit({"button": button, "request": True, "bytes": pack(button).hex(),
           "acc_available": avail, "acc_enabled": enabled,
           "set_speed_kph": extra if isinstance(extra, float) else None})

    # card polls the request at its own rate, so read back rather than assume: what the car reports after
    # the press is the only evidence that the button actually landed.
    time.sleep(settle_s)
    avail2, enabled2, extra2 = acc_state()
    moved = " (%+.1f)" % (extra2 - extra) if isinstance(extra, float) and isinstance(extra2, float) else ""
    return True, "requested %s, car now reports set=%s kph%s available=%s enabled=%s" % (
        button, _kph(extra2), moved, avail2, enabled2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--button", choices=sorted(BUTTONS), help="button to press")
    ap.add_argument("--frames", type=int, default=None,
                    help="frames to hold the button (100 Hz); card clamps to 1..40, default 6")
    ap.add_argument("--dry-run", action="store_true", help="check gates and print, request nothing")
    ap.add_argument("--state", action="store_true", help="print the car's ACC state and exit")
    ap.add_argument("--show-frames", action="store_true", help="print the packed bytes and exit")
    a = ap.parse_args()

    if a.show_frames:
        for b in BUTTONS:
            try:
                print("  %-9s %s" % (b, pack(b).hex()))
            except ValueError as exc:
                print("  %-9s %s" % (b, exc))
        return 0
    if a.state:
        ok, line = state_line()
        print(line)
        return 0 if ok else 1
    if not a.button:
        ap.error("--button is required unless --state or --show-frames is given")

    ok, sentence = press(a.button, dry_run=a.dry_run, frames=a.frames)
    print(sentence)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
