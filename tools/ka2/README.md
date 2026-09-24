# KA2 stock-ACC button control (and the vision → ACC bridge)

These are the box-side tools that let the settings app move this car's **stock ACC setpoint**, and let the
vision model slow the car for a bend using that same path. The car is a BYD Sealion 7 on a Kommu KA2
(bukapilot fork); its ACC is the factory system, openpilot longitudinal control is off.

Nothing here is a driving feature on its own. Read the traps before touching it.

## The pieces

| file | what it is |
| --- | --- |
| `ka2_acc_press.py` | one button press. Writes a request file; never transmits. |
| `ka2_vision_acc.py` | the vision → ACC bridge: steps the setpoint down for a bend the model sees, hands it back afterwards. Off unless enabled. |
| `ka2-vision-acc.service` | systemd unit for the bridge. Runs as `kommu` (it must read openpilot's msgq). |
| `bt_settings_service.py` | the box's settings service: serves every app row, including the tuning knobs, and takes the app's writes. |
| `ka2bt.service` | systemd unit for that service. The app's other end. |

Inside the car port, the other half lives in the repo: `opendbc/car/byd/acc_button.py` (frame builder) and a
hook in `selfdrive/car/card.py` that appends the frame to the CAN card already publishes.

## The two limits in the app

The bridge's two safety limits are ordinary live knobs, so they appear in the app next to the lane-correction
rows. No APK change is needed: the app renders whatever rows the box serves, and the service discovers each
row's range and shipped default by parsing `TUNING_LIMITS` out of the *deployed* file with `ast` (never
exec'd) - so a build that stopped reading the tuning file shows its rows as inert instead of pretending.

| row | key | range | shipped | effect |
| --- | --- | --- | --- | --- |
| Auto-slow floor (km/h) | `VIS_TURN_ACC_MIN_SETPOINT_KMH` | 30-90 | 30 | the bridge never steps the ACC setpoint below this. Raise only. |
| Auto-raise ceiling (km/h) | `VIS_TURN_ACC_MAX_RESTORE_KMH` | 60-130 | 130 | never hands the speed back above this, on top of never exceeding the setpoint you set. Lower only. |

Both ranges are one-sided on purpose: the file can only ever make the car do **less** than the committed
design. A file asking for floor 5 / ceiling 200 reads back as 30 / 130.

## Why it is shaped like this

* **card sends the press, never a helper process.** msgq allows exactly one publisher per topic and `sendcan`
  is card's: a helper publishing it makes msgq raise `MultiplePublishersError` inside card and card dies. That
  happened on 24 Sep 2026 and took the car daemon down mid-drive. The helper's only job now is to write
  `/data/hermes/acc/request` holding `"<button> <unix_ms> [frames]"`; card consumes and deletes it.
* **The message is not what the DBC says.** `0x3B0 PCM_BUTTONS` carries a 4-bit rolling counter in byte 6's high
  nibble and a checksum in byte 7 (all eight bytes sum to `0xFF`). Neither is in the DBC. Frames with zeros
  there are discarded, which cost two rounds of debugging.
* **Button meanings are not the obvious ones** (measured on this car, ACC engaged):
  * `SET` alone → setpoint **down** 5 km/h per press
  * `SET+RES` together → setpoint **up** 5 km/h per press (this is what the car's own speed rocker transmits)
  * `RES` alone → **cancels ACC**
  * at a standstill the `+` pattern behaves as resume/cancel, not as "increase"
* **Only the car's own bus matters.** The car transmits its own buttons on bus 0 and 2; the box sends on bus 0.
  When analysing logs, this fork also records car-originated frames flagged `src 0x80` — filter that out or you
  will attribute the car's presses to yourself.

## Gates (all re-checked by card, not just by the caller)

* ACC must be **engaged** (`cruiseState.enabled`), not merely switched on.
* One press per request; requests older than 3 s are ignored rather than fired late.
* Minimum 0.35 s between presses.
* `LKAS_ON_BTN` is never sent — that is the driver's lane-keep switch.
* Every press is appended to `/data/hermes/acc_presses.jsonl`.

## The vision → ACC bridge

Same geometry as `selfdrive/controls/lib/vision_turn_speed.py` on this fork (`path_curvature` / `find_bend` /
`v_cap = sqrt(a_lat / kappa)`), but actuated through the buttons because this car's longitudinal control is the
stock ACC. The branch version hands a deceleration request to the planner's MPC, which only helps a car where
openpilot works the pedals.

* Slowing outranks restoring in `decide()` — getting that order backwards makes the policy stop slowing after
  its first press.
* It raises back only up to the setpoint the driver had before the first press of that episode, and only when
  the road geometry allows the next step. That ceiling is the entire safety argument: it can undo its own work
  and nothing more.
* Bounds: max 3 steps (15 km/h) per bend, 2.5 s between presses, never below 30 km/h, never while below 25 km/h
  or with ACC disengaged.
* Every tuning range is one-sided, so `/data/hermes/tuning.json` can only make it act **less**.

### Enable / disable

```sh
# enable
python3 - <<'PY'
import json; p="/data/hermes/tuning.json"; t=json.load(open(p)); t["VIS_TURN_ACC_ENABLED"]=1
open(p+".tmp","w").write(json.dumps(t, indent=2, sort_keys=True)); import os; os.replace(p+".tmp", p)
PY
sudo systemctl enable --now ka2-vision-acc

# disable (either one, both take effect immediately; the flag is re-read every second)
sudo systemctl stop ka2-vision-acc
# or set VIS_TURN_ACC_ENABLED to 0 in the tuning file
```

Watch it: `journalctl -u ka2-vision-acc -f`. Audit: `/data/hermes/acc/vision_acc.jsonl` and
`/data/hermes/acc_presses.jsonl`.

### Validate before enabling

```sh
# what it would have done on a recorded drive - runs the live decide() logic, touches no car
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 ka2_vision_acc.py --replay <rlog.zst> --verbose

# on the car, pressing nothing
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 ka2_vision_acc.py --live --dry-run --log-every 2
```

Expect it to act rarely: on a 20 Sep drive it fired 0-1 times per segment, never more than two steps off, and
several segments slowed and then handed the speed back. A thin sample dominated by "ACC not engaged" means the
test drive was the limit, not the policy.
