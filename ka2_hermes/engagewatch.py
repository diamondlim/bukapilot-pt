import sys, time, json
sys.path.insert(0, '/data/openpilot')
from cereal.messaging import SubMaster

OUT = '/data/hermes/engagewatch.log'
sm = SubMaster(['carState', 'selfdriveState', 'onroadEvents', 'pandaStates', 'can'], ignore_alive=True)

def upd(k):
    u = sm.updated
    return bool(u(k)) if callable(u) else bool(u[k])

with open(OUT, 'a', buffering=1) as f:
    f.write(json.dumps({'mark': 'watch-start %s' % time.strftime('%Y-%m-%d %H:%M:%S')}) + '\n')
    t_end = time.time() + 3600
    while time.time() < t_end:
        sm.update(100)
        cs, ss = sm['carState'], sm['selfdriveState']
        tx = 0
        if upd('can'):
            tx = sum(1 for m in sm['can'] if int(m.src) & 0x80 and int(m.address) == 0x32E)
        try:
            ps = sm['pandaStates'][0]
            panda = '%s/%d/%s' % (ps.safetyModel, ps.safetyParam, ps.controlsAllowed)
        except Exception:
            panda = None
        f.write(json.dumps({
            't': time.strftime('%H:%M:%S'),
            'gear': str(cs.gearShifter), 'v': float(round(cs.vEgo * 3.6, 1)),
            'cruise': bool(cs.cruiseState.enabled), 'cruiseAvail': bool(cs.cruiseState.available),
            'set': float(round(cs.cruiseState.speed * 3.6, 1)),
            'state': str(ss.state), 'enabled': bool(ss.enabled), 'active': bool(ss.active),
            'alertType': str(ss.alertType), 'alert': str(ss.alertText1),
            'events': [str(e.name) for e in sm['onroadEvents']],
            'panda': panda, 'tx32e': tx,
        }) + '\n')
        time.sleep(0.5)
    f.write(json.dumps({'mark': 'watch-end'}) + '\n')
