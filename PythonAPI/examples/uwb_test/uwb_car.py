#!/usr/bin/env python3

# CARLA UWB sensor extension
# https://carla.org
#
# Author: dh0508 (GitHub: https://github.com/dh0508)
# Date: 2026
# Added UWB sensor to CARLA

import argparse
import random
import signal
import sys
import time

import carla


def parse_args():
    p = argparse.ArgumentParser(description='UWB: hero vehicle <- spawned vehicle')
    p.add_argument('--host', default='127.0.0.1', metavar='H')
    p.add_argument('-p', '--port', default=2000, type=int, metavar='P')
    p.add_argument('--range', default=150.0, type=float, metavar='M')
    return p.parse_args()


# ── NPC가 송신할 payload ────────────────────────────────────────────────────
# 이 함수가 반환하는 dict가 그대로 hero에게 전달됩니다.
# 원하는 데이터를 자유롭게 추가·제거하세요.
def build_npc_payload(npc, dist=0.0):
    v = npc.get_velocity()
    speed_kmh = round(3.6 * (v.x**2 + v.y**2 + v.z**2)**0.5, 1)
    return {
        'dist': f'{dist:.3f}',
        'speed_kmh': speed_kmh,
        'test': 203948203948,
        'test_text': "hello world",
        # 추가 데이터는 여기에
    }
# ──────────────────────────────────────────────────────────────────────────


def main():
    args = parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    tm = client.get_trafficmanager()

    actors = []

    def cleanup(sig=None, frame=None):
        print('\n[uwb_car] Cleaning up...')
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
        print('[uwb_car] ERROR: hero vehicle not found. Run manual_control.py first.')
        sys.exit(1)

    print(f'[uwb_car] Hero  id={hero.id}  type={hero.type_id}')

    # ── Spawn NPC vehicle ─────────────────────────────────────────────────
    vehicle_bps = list(bp_lib.filter('vehicle.tesla.model3')) or list(bp_lib.filter('vehicle.*'))
    npc = None
    for sp in random.sample(world.get_map().get_spawn_points(), 20):
        npc = world.try_spawn_actor(random.choice(vehicle_bps), sp)
        if npc:
            break

    if npc is None:
        print('[uwb_car] ERROR: could not spawn NPC vehicle.')
        sys.exit(1)

    actors.append(npc)
    npc.set_autopilot(True, tm.get_port())
    print(f'[uwb_car] NPC   id={npc.id}  type={npc.type_id}')

    # ── Attach UWB sensors ────────────────────────────────────────────────
    uwb_bp = bp_lib.find('sensor.other.uwb')
    uwb_bp.set_attribute('max_range',           str(args.range))
    uwb_bp.set_attribute('noise_los_stddev',    '0.05')
    uwb_bp.set_attribute('noise_nlos_bias_min', '0.2')
    uwb_bp.set_attribute('noise_nlos_bias_max', '2.0')
    uwb_bp.set_attribute('noise_nlos_stddev',   '0.3')

    hero_uwb = world.spawn_actor(uwb_bp, carla.Transform(carla.Location(z=1.5)), attach_to=hero)
    actors.append(hero_uwb)

    npc_uwb = world.spawn_actor(uwb_bp, carla.Transform(carla.Location(z=1.5)), attach_to=npc)
    actors.append(npc_uwb)

    print(f'[uwb_car] hero_uwb={hero_uwb.id}  npc_uwb={npc_uwb.id}  range={args.range} m')
    print('[uwb_car] Listening... Ctrl+C to stop.\n')

    def on_npc_uwb(measurement):
        dist = measurement[0].distance if len(measurement) > 0 else 0.0
        npc_uwb.send_uwb_payload(build_npc_payload(npc, dist))

    def on_hero_uwb(measurement):
        for det in measurement:
            p = det.payload or {}
            fields = '  '.join(f'{k}: {v}' for k, v in p.items())
            print(f'[hero←car] {fields}', flush=True)

    npc_uwb.listen(on_npc_uwb)
    hero_uwb.listen(on_hero_uwb)

    while True:
        time.sleep(0.1)
        if not hero.is_alive:
            cleanup()


if __name__ == '__main__':
    main()
