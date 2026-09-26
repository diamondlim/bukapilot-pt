#!/usr/bin/env python3
"""Wi-Fi control, so the app can move the box onto another network.

The box runs NetworkManager, so nmcli does the work. Two things matter more than the plumbing:

  * **A Wi-Fi password must never reach a log.** The service logs a session's opening commands, and the
    audit log records accepted changes, so the WIFI commands are the one place where a secret can travel
    through this protocol. `redact()` is applied to every logged line, and nmcli's own output is never
    echoed back to the phone (it can quote the password on failure).
  * **A SSID may contain a colon and a password may contain anything**, so nmcli's terse output is
    unescaped properly here rather than split on ":".
"""
import json
import subprocess

NMCLI = "/usr/bin/nmcli"
MAX_SSID = 64          # 802.11 limit, in bytes


def redact(line):
  """The line as it may be written to a log: a Wi-Fi password never goes in one."""
  text = (line or "").strip()
  if text.upper().startswith("WIFI CONNECT"):
    return "WIFI CONNECT [password redacted]"
  return text


def split_terse(line):
  """nmcli -t separates fields with ':' and escapes a literal ':' inside a value as '\\:'."""
  out, current, escaped = [], "", False
  for ch in line:
    if escaped:
      current += ch
      escaped = False
    elif ch == "\\":
      escaped = True
    elif ch == ":":
      out.append(current)
      current = ""
    else:
      current += ch
  out.append(current)
  return out


def nmcli(args, timeout=25):
  """Run nmcli. Returns stdout, or '' on any failure - callers report, they never raise."""
  try:
    done = subprocess.run([NMCLI] + list(args), capture_output=True, text=True, timeout=timeout)
    return done.stdout or ""
  except Exception:
    return ""


def wifi_status(ip_of):
  """The network the box is on now: what the phone would call it, and the address it holds."""
  status = {"ssid": "", "device": "", "ip": "", "signal": "", "connected": False}
  for line in nmcli(["-t", "-f", "NAME,DEVICE,TYPE", "connection", "show", "--active"]).splitlines():
    parts = split_terse(line)
    # nmcli names the type "802-11-wireless" (older builds say "wifi"); matching only the short form
    # silently produced an empty status while the list worked perfectly
    if len(parts) >= 3 and "wireless" in parts[2]:
      status["ssid"] = parts[0]
      status["device"] = parts[1]
      status["connected"] = True
  if status["device"]:
    status["ip"] = ip_of(status["device"]) or ""
    for line in nmcli(["-t", "-f", "IN-USE,SIGNAL", "device", "wifi", "list"]).splitlines():
      parts = split_terse(line)
      if len(parts) >= 2 and parts[0].strip() == "*":
        status["signal"] = parts[1]
  return status


def wifi_list():
  """Nearby networks, strongest first, one entry per SSID."""
  found = {}
  for line in nmcli(["-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE", "device", "wifi", "list"]).splitlines():
    parts = split_terse(line)
    if len(parts) < 4:
      continue
    ssid, signal, security, in_use = parts[0], parts[1], parts[2], parts[3]
    if not ssid:
      continue
    try:
      strength = int(signal)
    except Exception:
      strength = 0
    entry = {"ssid": ssid, "signal": strength,
             "sec": "" if security in ("", "--") else security,
             "in_use": in_use.strip() == "*"}
    if ssid not in found or strength > found[ssid]["signal"]:
      found[ssid] = entry
  return sorted(found.values(), key=lambda e: -e["signal"])


def wifi_connect(payload):
  """Join a network. The reply never contains nmcli's output, which can quote the password."""
  ssid = str(payload.get("ssid", "")).strip()
  password = str(payload.get("password", ""))
  if not ssid or len(ssid.encode("utf-8")) > MAX_SSID:
    return "ERR WIFI bad-ssid (1 to %d bytes)" % MAX_SSID
  if len(password) > 128:
    return "ERR WIFI bad-password"
  args = ["device", "wifi", "connect", ssid]
  if password:
    args += ["password", password]
  out = nmcli(args, timeout=45).lower()
  if "successfully activated" in out or "connection successfully activated" in out:
    return "OK WIFI ssid=%s connected" % ssid
  # No detail: an nmcli failure message can contain the password.
  return "ERR WIFI could-not-connect %s (check the password, or the network's security)" % ssid


def wifi_forget(ssid):
  """Drop a saved network, so a stale one cannot be picked up again by itself."""
  ssid = (ssid or "").strip()
  if not ssid:
    return "ERR WIFI bad-ssid"
  nmcli(["connection", "delete", ssid], timeout=25)
  return "OK WIFI ssid=%s forgotten" % ssid


def wifi_reply(command, payload, ip_of):
  """Every WIFI subcommand, returning the lines to send back."""
  if command == "STATUS":
    return ["W " + json.dumps(wifi_status(ip_of), separators=(",", ":"))]
  if command == "LIST":
    rows = wifi_list()
    out = ["W " + json.dumps(row, separators=(",", ":")) for row in rows]
    out.append("E %d" % len(rows))
    return out
  if command == "CONNECT":
    return [wifi_connect(payload or {})]
  if command == "FORGET":
    return [wifi_forget((payload or {}).get("ssid", ""))]
  return ["ERR WIFI unknown-subcommand (STATUS, LIST, CONNECT, FORGET)"]
