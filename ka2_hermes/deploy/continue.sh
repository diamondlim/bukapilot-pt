#!/usr/bin/bash

# --- Hermes: keep the tailnet up across reboots (added 2026-09-17) ---
if ! pgrep -x tailscaled >/dev/null 2>&1; then
  sudo sh -c "/data/tailscale/bin/tailscaled --state=/data/tailscale/state/tailscaled.state --socket=/data/tailscale/tailscaled.sock >>/data/tailscale/tailscaled.log 2>&1 &"
fi


# --- Hermes: block Kommu uploads while /data/hermes_pause_kommu exists ---
if [ -f /data/hermes_pause_kommu ]; then
  sudo iptables -N HERMES_BLOCK 2>/dev/null || true
  sudo iptables -C OUTPUT -j HERMES_BLOCK 2>/dev/null || sudo iptables -I OUTPUT 1 -j HERMES_BLOCK
  while read -r ip; do [ -n "$ip" ] && sudo iptables -A HERMES_BLOCK -d "$ip" -j REJECT; done < /data/hermes_pause_kommu
fi
# --- original vendor content below, unchanged ---
cd /data/openpilot

# --- Hermes: keep openpilot from wedging in selfdriveInitializing (added 2026-09-25) ---
# The box's updater hard-resets this branch (git reset --hard FETCH_HEAD) and wipes edits,
# so re-apply the two-file fix every boot, right before openpilot starts.
if [ -x /usr/kommu/hermes_ka2_patch.sh ]; then
  echo "hermes: re-applying carState rate-check patch";
  /usr/kommu/hermes_ka2_patch.sh || true
fi

./launch_openpilot.sh

# keep the uplink responsive on mobile hotspots
sudo iw dev wlan0 set power_save off 2>/dev/null
