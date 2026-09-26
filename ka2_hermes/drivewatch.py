import sys, time, json
sys.path.insert(0, '/data/openpilot')
from cereal.messaging import SubMaster

OUT = '/data/hermes/drivewatch.log'
WATCH = {0x32E: 'ACC_CMD', 0x32D: 'ACC_HUD', 0x3B0: 'PCM_BUTTONS', 0x1E2: 'STEER', 0x316: 'LKAS'}
sm = SubMaster(['carState', 'selfdriveState', 'onroadEvents', 'pandaStates', 'can'], ignore_alive=True)

def upd(k):
    u = sm.updated
    return bool(u(k)) if callable(u) else bool(u[k])

last_frame = {}
with open(OUT, 'a', buffering=1) as f:
    f.write(json.dumps({'mark': 'drivewatch-start %s' % time.strftime('%Y-%m-%d %H:%M:%S')}) + '\n')
    t_end = time.time() + 5400
    while time.time() < t_end:
        sm.update(100)
        cs, ss = sm['carState'], sm['selfdriveState']
        try:
            ps = sm['pandaStates'][0]
            panda = '%s/%d/%s' % (ps.safetyModel, ps.safetyParam, ps.controlsAllowed)
        except Exception:
            panda = None
        if upd('can'):
            for m in sm['can']:
                ad = int(m.address)
                if ad in WATCH and (int(m.src) & 0x80):
                    d = bytes(m.dat).hex()
                    if last_frame.get(ad) != d:
                        last_frame[ad] = d
                        f.write(json.dumps({'t': time.strftime('%H:%M:%S.%f')[:-4], 'can_tx': WATCH[ad],
                                            'addr': '0x%03X' % ad, 'd': d}) + '\n')
        f.write(json.dumps({
            't': time.strftime('%H:%M:%S'),
            'v': float(round(cs.vEgo * 3.6, 1)), 'gear': str(cs.gearShifter),
            'canValid': bool(cs.canValid), 'canTimeout': bool(cs.canTimeout),
            'acc': bool(cs.cruiseState.enabled), 'accAvail': bool(cs.cruiseState.available),
            'set': float(round(cs.cruiseState.speed * 3.6, 1)),
            'brake': bool(cs.brakePressed), 'gas': bool(cs.gasPressed),
            'steer': float(round(cs.steeringAngleDeg, 1)),
            'op': str(ss.state), 'en': bool(ss.enabled), 'act': bool(ss.active),
            'alert': str(ss.alertText1), 'alertType': str(ss.alertType),
            'events': [str(e.name) for e in sm['onroadEvents']],
            'panda': panda,
        }) + '\n')
        time.sleep(0.4)
    f.write(json.dumps({'mark': 'drivewatch-end'}) + '\n')
