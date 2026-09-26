#!/usr/bin/env python3
"""Bluetooth settings endpoint for the KA2 (SPP / RFCOMM over BlueZ 5.72).

BlueZ 5.7x refuses service records registered the old way (`sdptool add SP` silently does
nothing), so this registers org.bluez.Profile1 over D-Bus with the SPP UUID instead.

Line protocol on the serial link:
    LIST                        -> the writable keys and their current values
    GET <Key>                   -> value of one key
    SET <Key> <value>           -> write it (whitelist + range checked, always audited)
    HELP                        -> this text
Only keys in ALLOW can be read or written, values are range-checked, every change is appended
to /data/hermes/bt_audit.log (never /var/log: the kommu user cannot write there).
"""
import os, re, socket, time, sys
import dbus, dbus.service, dbus.mainloop.glib
from gi.repository import GLib

PARAMS = "/data/params/d"
AUDIT = "/data/hermes/bt_audit.log"
SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"
ADAPTER_PATH = "/org/bluez/hci0"

# whitelist: key -> (lo, hi) or None for a free string. Nothing outside this is reachable.
ALLOW = {
    "LaneCorrectionGain": (0.0, 2.0),
    "NetworkMetered": None,
    "ExperimentalMode": None,
    "LongitudinalPersonality": (0.0, 3.0),
    "SpeedLimitControl": None,
}

os.makedirs(os.path.dirname(AUDIT), exist_ok=True)


def log(msg):
    line = "%s %s" % (time.strftime("%Y-%m-%dT%H:%M:%S"), msg)
    print(line, flush=True)
    with open(AUDIT, "a") as fh:
        fh.write(line + "\n")


def read_key(key):
    try:
        with open(os.path.join(PARAMS, key)) as fh:
            return fh.read().strip()
    except Exception:
        return ""


def write_key(key, value):
    if key not in ALLOW:
        return "ERR not-whitelisted"
    lo_hi = ALLOW[key]
    if lo_hi is not None:
        try:
            v = float(value)
        except ValueError:
            return "ERR not-a-number"
        if not (lo_hi[0] <= v <= lo_hi[1]):
            return "ERR out-of-range %s..%s" % lo_hi
        value = str(v)
    if not re.fullmatch(r"[A-Za-z0-9_.+-]{1,32}", value):
        return "ERR bad-value"
    try:
        with open(os.path.join(PARAMS, key), "w") as fh:
            fh.write(value)
    except Exception as exc:
        return "ERR write-failed %s" % exc
    log("SET %s = %s" % (key, value))
    return "OK %s = %s" % (key, value)


def handle(line):
    parts = line.strip().split()
    if not parts:
        return ""
    cmd = parts[0].upper()
    if cmd == "HELP":
        return "commands: LIST | GET <Key> | SET <Key> <value> | HELP"
    if cmd == "LIST":
        return "\n".join("%s=%s" % (k, read_key(k)) for k in sorted(ALLOW))
    if cmd == "GET" and len(parts) == 2:
        return "ERR not-whitelisted" if parts[1] not in ALLOW else "%s=%s" % (parts[1], read_key(parts[1]))
    if cmd == "SET" and len(parts) >= 3:
        return write_key(parts[1], " ".join(parts[2:]))
    return "ERR usage"


class Profile(dbus.service.Object):
    @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}", out_signature="")
    def NewConnection(self, device, fd, props):
        sock = socket.socket(fileno=fd.take())
        log("connect from %s" % device)
        try:
            sock.settimeout(300)
            buf = b""
            sock.sendall(b"KA2 settings ready. Type HELP\n")
            while True:
                chunk = sock.recv(1024)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    reply = handle(line.decode("utf-8", "replace"))
                    if reply:
                        sock.sendall((reply + "\n").encode())
        except Exception as exc:
            log("session ended: %s" % exc)
        finally:
            sock.close()
        log("disconnect")

    @dbus.service.method("org.bluez.Profile1", in_signature="", out_signature="")
    def Release(self):
        log("release")

    @dbus.service.method("org.bluez.Profile1", in_signature="o", out_signature="")
    def RequestDisconnection(self, device):
        log("request-disconnection %s" % device)


AGENT_PATH = "/com/hermes/ka2settings/agent"


class Agent(dbus.service.Object):
    """Pairing agent. NoInputNoOutput = this box has no keypad or display, which is what a
    headless device must advertise; pairing then completes without a prompt on the box side."""

    @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
    def Release(self):
        log("agent: release")

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        log("agent: pin code for %s" % device)
        return "0000"

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        log("agent: passkey for %s" % device)
        return dbus.UInt32(0)

    @dbus.service.method("org.bluez.Agent1", in_signature="ouq", out_signature="")
    def DisplayPasskey(self, device, passkey, entered):
        log("agent: display passkey %s" % device)

    @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pincode):
        log("agent: display pin %s" % device)

    @dbus.service.method("org.bluez.Agent1", in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        log("agent: confirmed %s" % device)

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        log("agent: authorized %s" % device)

    @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        log("agent: authorized service %s for %s" % (uuid, device))

    @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
    def Cancel(self):
        log("agent: cancel")


def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    path = "/com/hermes/ka2settings"
    profile = Profile(bus, path)
    opts = {"Name": "KA2 Settings", "Role": "server", "Channel": dbus.UInt16(1),
            "RequireAuthentication": dbus.Boolean(True), "RequireAuthorization": dbus.Boolean(True),
            "Service": SPP_UUID}
    try:
        mgr = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"), "org.bluez.ProfileManager1")
        mgr.RegisterProfile(path, SPP_UUID, opts)
        log("registered SPP profile at %s" % path)
    except Exception as exc:
        log("REGISTER FAILED: %s" % exc)
        return 1
    agent_obj = Agent(bus, AGENT_PATH)
    try:
        amgr = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"), "org.bluez.AgentManager1")
        amgr.RegisterAgent(AGENT_PATH, "NoInputNoOutput")
        amgr.RequestDefaultAgent(AGENT_PATH)
        log("pairing agent registered (NoInputNoOutput, default)")
    except Exception as exc:
        log("AGENT FAILED: %s" % exc)
    # Adapter state, re-applied on every start: a discoverable timeout of 180 s (the default)
    # is why the box kept disappearing from scans, and hciconfig's class does not survive a
    # bluetoothd restart. Setting them here means the unit owns them.
    try:
        props = dbus.Interface(bus.get_object("org.bluez", ADAPTER_PATH), "org.freedesktop.DBus.Properties")
        for name, value in (("Powered", dbus.Boolean(True)), ("Discoverable", dbus.Boolean(True)),
                            ("Pairable", dbus.Boolean(True)), ("DiscoverableTimeout", dbus.UInt32(0))):
            props.Set("org.bluez.Adapter1", name, value)
        log("adapter: discoverable, pairable, timeout 0")
    except Exception as exc:
        log("AGENT FAILED: %s" % exc)
    GLib.MainLoop().run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
