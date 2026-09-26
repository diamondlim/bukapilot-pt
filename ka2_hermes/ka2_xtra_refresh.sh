#!/bin/bash
# Keep the modem's gpsOneXTRA predicted orbits loaded.
#
# Why: the assistance data is valid for 7 days (the module reports 10080 minutes) and the vendor note says
# time and file must be re-injected after the module is powered off. The box loses power with the car, so
# without this the receiver would spend most of its life with no assistance data - which is exactly the
# state it was in until now.
set -u
LOG=/data/hermes/logs/ka2_xtra_refresh.log
TUNE=/tmp/ka2_xtra_inject.py
FILE=/tmp/xtra2.bin
URL=http://xtrapath4.izatcloud.net/xtra2.bin
mkdir -p "$(dirname "$LOG")"

stamp() { date -Is; }
say() { echo "$(stamp) $* " >> "$LOG"; }

[ -x /usr/bin/python3 ] || true
if [ ! -f "$TUNE" ]; then say "injector missing at $TUNE - nothing done"; exit 0; fi

valid=$(sudo -n mmcli -m 0 --command 'AT+QGPSXTRADATA?' 2>/dev/null | sed -n "s/.*+QGPSXTRADATA: \([0-9]*\),\"\([^\"]*\)\".*/\1 \2/p")
minutes=${valid%% *}
injected=${valid#* }
if [ -n "${minutes:-}" ] && [ "$minutes" -gt 2880 ]; then
  say "still valid: $minutes minutes left, injected $injected - no action"
  exit 0
fi
say "assistance data low or absent (${valid:-no answer}) - refreshing"

curl -s -m 120 -o "$FILE" "$URL" || { say "download failed"; exit 0; }
size=$(stat -c %s "$FILE" 2>/dev/null || echo 0)
[ "$size" -gt 10000 ] || { say "downloaded file too small ($size bytes)"; exit 0; }

sudo -n systemctl stop ka2-gnss-at.service
out=$(python3 "$TUNE" --file "$FILE" --remote RAM:xtra2.bin 2>&1)
sudo -n systemctl start ka2-gnss-at.service
echo "$out" >> "$LOG"
if echo "$out" | grep -q "orbit data injected and held"; then
  say "OK: re-injected $size bytes of predicted orbits"
else
  say "FAILED to inject - see above"
fi

