#!/usr/bin/env bash
# Install the GNSS bring-up and the NMEA publisher as services on the KA2. Runs ON THE BOX as root.
#
#   ka2_gnss_install.sh [--port /dev/ttyS3] [--baud 9600] [--from-verdict]
#
# With --from-verdict (the default when no --port is given) the port and speed come from the bring-up's
# own verdict file, so nothing is guessed: if the sweep found no NMEA, this refuses to install a
# publisher that would sit there logging silence, and says so.
#
# Two units, because the two halves need different privileges: the bring-up drives sysfs GPIO (root) and
# the publisher must run as kommu under the fork venv, since cereal needs capnp that the system python
# does not have.
set -euo pipefail

VERDICT=${VERDICT:-/dev/shm/ka2_gnss_find.json}
DIR=/data/hermes
LOGDIR=$DIR/logs
PORT=""
BAUD=""

while [ $# -gt 0 ]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --baud) BAUD="$2"; shift 2 ;;
    --from-verdict) FROM_VERDICT=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [ -z "$PORT" ]; then
  FROM_VERDICT=1
  if [ ! -s "$VERDICT" ]; then
    echo "no verdict at $VERDICT yet - run ka2_gnss_bringup.py first" >&2
    exit 3
  fi
  read -r PORT BAUD < <(/usr/bin/python3 - "$VERDICT" <<'PY'
import json, sys
v = json.load(open(sys.argv[1]))
hits = v.get("nmea") or []
if hits:
    best = max(hits, key=lambda h: h.get("sentences", 0))
    print(best["port"], best["baud"])
PY
)
  if [ -z "${PORT:-}" ]; then
    echo "the sweep found no NMEA on any port - not installing a publisher that would log silence." >&2
    echo "that is a hardware question (antenna, sky view, or which power line the vendor script names)." >&2
    exit 4
  fi
fi
BAUD=${BAUD:-9600}
echo "using $PORT at $BAUD baud"

mkdir -p "$LOGDIR"
chown -R kommu:kommu "$LOGDIR" 2>/dev/null || true

# --- the bring-up, at boot, as root: the chip is dark until its lines are driven ------------------
cat >/etc/systemd/system/ka2-gnss-bringup.service <<EOF
[Unit]
Description=KA2 GNSS module power/reset bring-up
After=local-fs.target
Before=ka2-gnss-pub.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/python3 $DIR/ka2_gnss_bringup.py --json $VERDICT
StandardOutput=append:$LOGDIR/ka2_gnss_bringup.log
StandardError=append:$LOGDIR/ka2_gnss_bringup.log
# The sweep dwells a few seconds on every candidate port; it must not be killed mid-run.
TimeoutStartSec=600
EOF

# --- the publisher: steady state, as kommu, under the fork venv -----------------------------------
cat >/etc/systemd/system/ka2-gnss-pub.service <<EOF
[Unit]
Description=KA2 GNSS NMEA -> gpsLocationExternal bridge
After=ka2-gnss-bringup.service
Requires=ka2-gnss-bringup.service

[Service]
Type=simple
User=kommu
Group=kommu
WorkingDirectory=$DIR
ExecStart=/usr/local/venv/bin/python3 $DIR/ka2_gnss_pub.py --port $PORT --baud $BAUD
Restart=always
RestartSec=5
# Serial reads only: no privileges, and no need to touch the bus beyond publishing.
NoNewPrivileges=true
StandardOutput=append:$LOGDIR/ka2_gnss_pub.log
StandardError=append:$LOGDIR/ka2_gnss_pub.log
EOF

systemctl daemon-reload
systemctl enable ka2-gnss-bringup.service ka2-gnss-pub.service >/dev/null
systemctl restart ka2-gnss-bringup.service
systemctl restart ka2-gnss-pub.service
sleep 6
systemctl --no-pager --lines=3 status ka2-gnss-pub.service || true
echo
echo "--- what the box now publishes (20 s) ---"
su - kommu -c "cd /data/openpilot && /usr/local/venv/bin/python3 $DIR/ka2_gps_verify.py --seconds 20" || \
  echo "verify returned non-zero: see $LOGDIR/ka2_gnss_pub.log"
echo
echo "log tail:"; tail -5 "$LOGDIR/ka2_gnss_pub.log" 2>/dev/null || true
