#!/usr/bin/env python3
"""Is a Controls Mismatch condition present on the car right now, and which of its three causes is it?

selfdrived raises EventName.controlsMismatch - an IMMEDIATE_DISABLE alert, so openpilot drops out the
moment it appears - when any of these is true:

  1. the panda's reported safetyModel / safetyParam / alternativeExperience does not match the safety
     configuration the car port declared,
  2. the panda reports safetyRxChecksInvalid (CAN frames arrived before the safety was applied - a
     startup race, cleared by re-initialising the panda),
  3. openpilot has been enabled for 200 frames (2 s) while the panda reports controlsAllowed = False.

This reads all three off the car, so the cause is named rather than guessed at.
"""
import sys

sys.path.insert(0, "/data/openpilot")
import cereal.messaging as messaging  # noqa: E402
from cereal import car  # noqa: E402
from openpilot.common.params import Params  # noqa: E402

sm = messaging.SubMaster(["pandaStates", "selfdriveState", "carState"])

seen = False
for _ in range(60):
  sm.update(200)
  if sm.updated["pandaStates"]:
    seen = True
    break

if not seen:
  print("no pandaStates on the bus after 12 s")
  print("  alive: %s" % {k: v for k, v in sm.alive.items()})
  print("  valid: %s" % {k: v for k, v in sm.valid.items()})
  sys.exit(1)

# CarParams is published once at startup, so a late subscriber never sees it: read the device's own
# cached copy instead
raw = Params().get("CarParams")
if not raw:
  print("no cached CarParams on the device")
  sys.exit(1)

with car.CarParams.from_bytes(raw) as cp:
  configs = [(str(c.safetyModel), int(c.safetyParam)) for c in cp.safetyConfigs]
  alt_exp = int(cp.alternativeExperience)
  passive = bool(cp.passive)
  long_ctl = bool(cp.openpilotLongitudinalControl)

print("car port declares:")
for i, cfg in enumerate(configs):
  print("  [%d] safetyModel=%s safetyParam=%s" % (i, cfg[0], cfg[1]))
print("  alternativeExperience=%d  passive=%s  openpilotLongitudinalControl=%s"
      % (alt_exp, passive, long_ctl))

print()
print("the panda reports:")
for i, ps in enumerate(sm["pandaStates"]):
  print("  [%d] safetyModel=%s safetyParam=%s alternativeExperience=%s"
        % (i, ps.safetyModel, ps.safetyParam, ps.alternativeExperience))
  print("      controlsAllowed=%s  safetyRxChecksInvalid=%s"
        % (ps.controlsAllowed, ps.safetyRxChecksInvalid))

print()
verdict = []
for i, ps in enumerate(sm["pandaStates"]):
  if i >= len(configs):
    verdict.append("cause 1: panda %d has no declared config to compare with" % i)
    continue
  declared_model, declared_param = configs[i]
  problems = []
  if str(ps.safetyModel) != declared_model:
    problems.append("safetyModel %s vs declared %s" % (ps.safetyModel, declared_model))
  if int(ps.safetyParam) != declared_param:
    problems.append("safetyParam %s vs declared %s" % (ps.safetyParam, declared_param))
  if int(ps.alternativeExperience) != alt_exp:
    problems.append("alternativeExperience %s vs declared %s" % (ps.alternativeExperience, alt_exp))
  verdict.append("cause 1 (safety config mismatch), panda %d: %s"
                 % (i, "; ".join(problems) if problems else "no - they match"))
  verdict.append("cause 2 (rx checks invalid), panda %d: %s"
                 % (i, "YES" if ps.safetyRxChecksInvalid else "no"))

ss = sm["selfdriveState"]
cs = sm["carState"]
verdict.append("cause 3 (enabled while the panda refuses controls): enabled=%s active=%s allowed=%s"
               % (ss.enabled, ss.active, [ps.controlsAllowed for ps in sm["pandaStates"]]))
print("\n".join(verdict))
print()
print("alert text now: %r / %r   (vEgo %.2f m/s, standstill=%s)"
      % (ss.alertText1, ss.alertText2, cs.vEgo, cs.standstill))
