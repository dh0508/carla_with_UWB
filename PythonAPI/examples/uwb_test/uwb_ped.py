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
from carla import command


# ── 보행자가 송신할 payload ─────────────────────────────────────────────────
_prev_loc = None
_prev_t   = None

def build_ped_payload(walker, dist=0.0):
    global _prev_loc, _prev_t
    now = time.time()
    loc = walker.get_location()

    if _prev_loc is not None and _prev_t is not None:
        dt = now - _prev_t
        if dt > 0:
            dx = loc.x - _prev_loc.x
            dy = loc.y - _prev_loc.y
            dz = loc.z - _prev_loc.z
            speed_ms = round((dx**2 + dy**2 + dz**2)**0.5 / dt, 2)
        else:
            speed_ms = 0.0
    else:
        speed_ms = 0.0

    _prev_loc = loc
    _prev_t   = now

    return {
        'dist': f'{dist:.3f}',
        'speed_ms': speed_ms,
        'est': 203948203948,
        'test_text': "hello world",
    }
# ──────────────────────────────────────────────────────────────────────────


def parse_args():
    p = argparse.ArgumentParser(description='UWB: hero vehicle <- pedestrian')
    p.add_argument('--host', default='127.0.0.1', metavar='H')
    p.add_argument('-p', '--port', default=2000, type=int, metavar='P')
    p.add_argument('--range', default=150.0, type=float, metavar='M')
    p.add_argument('--speed', default=1.4, type=float, metavar='S')
    return p.parse_args()


def main():
    args = parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    all_id = []        # [con_id, walker_id]
    all_actors = None
    uwb_actors = []

    def cleanup(sig=None, frame=None):
        print('\n[uwb_ped] Cleaning up...')
        for a in reversed(uwb_actors):
            try:
                if a.is_alive:
                    a.destroy()
            except Exception:
                pass
        if all_actors:
            # stop controller (index 0), then destroy both
            try:
                all_actors[0].stop()
            except Exception:
                pass
        if all_id:
            client.apply_batch_sync([command.DestroyActor(i) for i in all_id])
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
        print('[uwb_ped] ERROR: hero vehicle not found. Run manual_control.py first.')
        sys.exit(1)

    print(f'[uwb_ped] Hero  id={hero.id}  type={hero.type_id}')

    # ── 1. Random spawn location from navigation mesh ─────────────────────
    spawn_point = carla.Transform()
    loc = world.get_random_location_from_navigation()
    if loc is None:
        print('[uwb_ped] ERROR: no walkable location found.')
        sys.exit(1)
    spawn_point.location = loc

    # ── 2. Spawn walker (do_tick=True) ────────────────────────────────────
    walker_bp = random.choice(list(bp_lib.filter('walker.pedestrian.*')))
    if walker_bp.has_attribute('is_invincible'):
        walker_bp.set_attribute('is_invincible', 'false')

    walker_speed = args.speed
    if walker_bp.has_attribute('speed'):
        walker_speed = float(walker_bp.get_attribute('speed').recommended_values[1])

    results = client.apply_batch_sync([command.SpawnActor(walker_bp, spawn_point)], True)
    if not results or results[0].error:
        print(f'[uwb_ped] ERROR spawning pedestrian: {results[0].error if results else "no result"}')
        sys.exit(1)
    walker_id = results[0].actor_id

    # ── 3. Spawn controller (do_tick=True) ────────────────────────────────
    controller_bp = bp_lib.find('controller.ai.walker')
    results = client.apply_batch_sync(
        [command.SpawnActor(controller_bp, carla.Transform(), walker_id)], True)
    if not results or results[0].error:
        print(f'[uwb_ped] ERROR spawning controller: {results[0].error if results else "no result"}')
        client.apply_batch_sync([command.DestroyActor(walker_id)])
        sys.exit(1)
    controller_id = results[0].actor_id

    # ── 4. Get actors in generate_traffic order: [controller, walker] ─────
    all_id = [controller_id, walker_id]
    all_actors = world.get_actors(all_id)

    # wait for a tick (generate_traffic.py 패턴 그대로)
    world.wait_for_tick()

    # ── 5. Start AI (generate_traffic.py 패턴 그대로 — wait 없음) ──────────
    world.set_pedestrians_cross_factor(0.0)
    all_actors[0].start()
    all_actors[0].go_to_location(world.get_random_location_from_navigation())
    all_actors[0].set_max_speed(walker_speed)

    walker = all_actors[1]
    print(f'[uwb_ped] Pedestrian id={walker.id}  spawn={spawn_point.location}')

    # ── Attach UWB sensors ────────────────────────────────────────────────
    uwb_bp = bp_lib.find('sensor.other.uwb')
    uwb_bp.set_attribute('max_range',           str(args.range))
    uwb_bp.set_attribute('noise_los_stddev',    '0.05')
    uwb_bp.set_attribute('noise_nlos_bias_min', '0.2')
    uwb_bp.set_attribute('noise_nlos_bias_max', '2.0')
    uwb_bp.set_attribute('noise_nlos_stddev',   '0.3')

    hero_uwb = world.spawn_actor(uwb_bp, carla.Transform(carla.Location(z=1.5)), attach_to=hero)
    uwb_actors.append(hero_uwb)

    ped_uwb = world.spawn_actor(uwb_bp, carla.Transform(carla.Location(z=1.0)), attach_to=walker)
    uwb_actors.append(ped_uwb)

    print(f'[uwb_ped] hero_uwb={hero_uwb.id}  ped_uwb={ped_uwb.id}  range={args.range} m')
    print('[uwb_ped] Listening... Ctrl+C to stop.\n')

    def on_ped_uwb(measurement):
        dist = measurement[0].distance if len(measurement) > 0 else 0.0
        ped_uwb.send_uwb_payload(build_ped_payload(walker, dist))

    def on_hero_uwb(measurement):
        for det in measurement:
            p = det.payload or {}
            fields = '  '.join(f'{k}: {v}' for k, v in p.items())
            print(f'[hero←ped] {fields}', flush=True)

    ped_uwb.listen(on_ped_uwb)
    hero_uwb.listen(on_hero_uwb)

    # ── Main loop (generate_traffic.py 패턴 그대로) ────────────────────────
    while True:
        world.wait_for_tick()

        if not hero.is_alive or not walker.is_alive:
            cleanup()


if __name__ == '__main__':
    main()
