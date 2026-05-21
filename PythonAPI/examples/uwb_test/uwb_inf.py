#!/usr/bin/env python3

# CARLA UWB sensor extension
# https://carla.org
#
# Author: dh0508 (GitHub: https://github.com/dh0508)
# Date: 2026
# Added UWB sensor to CARLA

import argparse
import signal
import sys
import time

import carla

STATE_MAP = {
    carla.TrafficLightState.Red:     'Red',
    carla.TrafficLightState.Yellow:  'Yellow',
    carla.TrafficLightState.Green:   'Green',
    carla.TrafficLightState.Off:     'Off',
    carla.TrafficLightState.Unknown: 'Unknown',
}


# ── 신호등이 송신할 payload ─────────────────────────────────────────────────
# 이 함수가 반환하는 dict가 그대로 hero에게 전달됩니다.
# 원하는 데이터를 자유롭게 추가·제거하세요.
def build_tl_payload(tl, dist=0.0):
    return {
        'dist': f'{dist:.3f}',
        'signal': STATE_MAP.get(tl.get_state(), 'Unknown'),
        'test': 203948203948,
        'test_text': "git test",
    }
# ──────────────────────────────────────────────────────────────────────────


def parse_args():
    p = argparse.ArgumentParser(description='UWB: hero vehicle <- traffic light')
    p.add_argument('--host', default='127.0.0.1', metavar='H')
    p.add_argument('-p', '--port', default=2000, type=int, metavar='P')
    p.add_argument('--range', default=150.0, type=float, metavar='M')
    return p.parse_args()


def main():
    args = parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    actors = []

    def cleanup(sig=None, frame=None):
        print('\n[uwb_inf] Cleaning up...')
        for a in reversed(actors):
            try:
                if a.is_alive:
                    if hasattr(a, 'stop'):
                        a.stop()
                    a.destroy()
            except Exception:
                pass
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ── Find hero vehicle ─────────────────────────────────────────────────
    hero = None
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == 'hero':
            hero = actor
            break

    if hero is None:
        print('[uwb_inf] ERROR: hero vehicle not found. Run manual_control.py first.')
        sys.exit(1)

    print(f'[uwb_inf] Hero  id={hero.id}  type={hero.type_id}')

    # ── Find closest traffic light to hero ────────────────────────────────
    tl_actors = list(world.get_actors().filter('traffic.traffic_light'))
    if not tl_actors:
        print('[uwb_inf] ERROR: no traffic lights in this map.')
        sys.exit(1)

    hero_loc = hero.get_location()
    tl = min(tl_actors, key=lambda a: a.get_location().distance(hero_loc))
    tl_loc = tl.get_location()
    print(f'[uwb_inf] Traffic light id={tl.id}  '
          f'pos=({tl_loc.x:.1f}, {tl_loc.y:.1f})  '
          f'dist={hero_loc.distance(tl_loc):.1f} m')

    # ── Attach UWB sensors ────────────────────────────────────────────────
    uwb_bp = bp_lib.find('sensor.other.uwb')
    uwb_bp.set_attribute('max_range',           str(args.range))
    uwb_bp.set_attribute('noise_los_stddev',    '0.03')
    uwb_bp.set_attribute('noise_nlos_bias_min', '0.1')
    uwb_bp.set_attribute('noise_nlos_bias_max', '1.0')
    uwb_bp.set_attribute('noise_nlos_stddev',   '0.2')

    hero_uwb = world.spawn_actor(uwb_bp, carla.Transform(carla.Location(z=1.5)), attach_to=hero)
    actors.append(hero_uwb)

    tl_uwb = world.spawn_actor(uwb_bp, carla.Transform(carla.Location(z=3.0)), attach_to=tl)
    actors.append(tl_uwb)

    print(f'[uwb_inf] hero_uwb={hero_uwb.id}  tl_uwb={tl_uwb.id}  range={args.range} m')
    print('[uwb_inf] Listening... Ctrl+C to stop.\n')

    def on_tl_uwb(measurement):
        dist = measurement[0].distance if len(measurement) > 0 else 0.0
        tl_uwb.send_uwb_payload(build_tl_payload(tl, dist))

    def on_hero_uwb(measurement):
        for det in measurement:
            p = det.payload or {}
            fields = '  '.join(f'{k}: {v}' for k, v in p.items())
            print(f'[hero←tl]  {fields}', flush=True)

    tl_uwb.listen(on_tl_uwb)
    hero_uwb.listen(on_hero_uwb)

    while True:
        time.sleep(0.1)
        if not hero.is_alive:
            cleanup()


if __name__ == '__main__':
    main()
