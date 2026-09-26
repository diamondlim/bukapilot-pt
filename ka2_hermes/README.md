# KA2 box-side tooling

Custom code that runs on the KommuAssist KA2 fitted to a BYD Sealion 7 (bukapilot fork). None of it is part
of openpilot, and the fork's updater never installs it: it lives on the box under `/data/hermes` and is
started by systemd units. This directory is the versioned copy, so the box can be rebuilt from this repo
plus the tracked fork branch.

## Active

- **`ka2_vision_acc.py`** + **`ka2_acc_press.py`** — the vision -> stock-ACC bridge
  (`ka2-vision-acc.service`). It watches `modelV2` and lowers the ACC *setpoint* with button presses when a
  bend ahead or a car ahead demands less speed, then hands the speed back. It never actuates the brakes: on
  this platform the setpoint is the only lever the box has, and the car's own ACC does the decelerating.
  Every behaviour is a live key in `/data/hermes/tuning.json`; the press tool owns its own gates and audit.
- **`bt_settings_service.py`** + **`wifi_control.py`** — the phone app's settings service
  (`ka2bt.service`), including every live tuning row the app shows. It reads each knob's range and shipped
  default out of the source files themselves (`ast`, never `exec`), so a row appears only for a knob the
  deployed code actually reads, and a half-wired knob silently disappears instead of lying.
- **`hermes_ka2_patch.sh`** — re-applied at every boot (`hermes-ka2-patch.service`, the timer, and a line in
  `/data/continue.sh`) so the openpilot tree stays patched even after the updater hard-resets it. The
  patches themselves are committed on the tracked branch now, so this is the fallback for a branch change
  rather than the only copy.
- **`deploy/`** — the units and the `/data/continue.sh` hook, kept for rebuilding the box.

## Diagnostics (run by hand, read-only)

`drivewatch.py`, `engagewatch.py`, `acc_decode.py`, `log_summary.py`, `route_gps.py`, `centering_study.py`,
`ka2_acc_watch_press.py`, `probe_*.py` — recorders and probes used to establish what the car, the CAN bus
and the box were doing during a drive.

## Historical

`ka2_gnss_*.py|sh`, `ka2_pose_pub.py`, `ka2_xtra_refresh.sh` — an earlier attempt to feed this box its own
GNSS and pose; superseded by the fork-side fixes. `bt_settings_service.v1.py` and
`bt_settings_service.pre-acc.py` — earlier revisions, kept for comparison.

## Tuning keys

- Bend auto-slow: `VIS_TURN_ACC_{ENABLED, MIN_SETPOINT_KMH, MAX_RESTORE_KMH, MAX_STEPS, COOLDOWN_S,
  TRIGGER_S, LOOKAHEAD_MAX, A_LAT, MIN_RADIUS, MIN_V_KMH, MARGIN_KMH, RESTORE_MARGIN_KMH, RESTORE}`.
- Car ahead: `VIS_LEAD_ACC_{ENABLED, LOOKAHEAD_M, MARGIN_KMH, MIN_PROB, MAX_STEPS}` — ships **off**.

Every key is re-read about once a second; a missing file or an unreadable value falls back to the last good
value, and every shipped default is the least aggressive setting in its range. The auto-slow floor is the
hard bound on how far the setpoint can be walked down.

## Rebuilding the box

1. Clone the tracked fork branch — the openpilot-side fixes (rate/alive gating, the radar assumption) are
   already committed there.
2. Copy the `*.py` files to `/data/hermes`, `chown kommu:kommu`, mode 644.
3. Copy `hermes_ka2_patch.sh` to `/usr/kommu/`, mode 755.
4. Install `deploy/*.service` and the timer, `systemctl daemon-reload`, then enable the timer and both
   services.
5. Add the hook line to `/data/continue.sh` immediately before `./launch_openpilot.sh`.
