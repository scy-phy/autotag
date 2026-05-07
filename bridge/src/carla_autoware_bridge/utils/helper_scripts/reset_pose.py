#!/usr/bin/env python3

import carla
import argparse
import time


def find_ego_vehicle(world, role_names):
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') in role_names:
            return actor
    return None


def main():
    parser = argparse.ArgumentParser(description="Reset Ego Vehicle Pose in CARLA")
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=1403)

    # The default pose here is the same as in the objects.json file 
    parser.add_argument('--x', type=float, default=-54.344658)
    parser.add_argument('--y', type=float, default=137.050995)
    parser.add_argument('--z', type=float, default=0.6)
    parser.add_argument('--yaw', type=float, default=0.352127)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--roll', type=float, default=0.0)

    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    world = client.get_world()

    time.sleep(0.3)  # Wait a moment for the world to be ready

    print("Searching for ego vehicle...")

    ego_vehicle = find_ego_vehicle(
        world,
        ['ego_vehicle', 'hero', 'hero0', 'hero1']
    )

    if ego_vehicle is None:
        print("Ego vehicle not found.")
        return

    print(f"Ego vehicle found. Actor ID: {ego_vehicle.id}")
    print("Current transform:", ego_vehicle.get_transform())

    # Stop vehicle motion
    ego_vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    ego_vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))

    time.sleep(0.2)

    new_transform = carla.Transform(
        carla.Location(x=args.x, y=args.y, z=args.z),
        carla.Rotation(
            pitch=args.pitch,
            yaw=args.yaw,
            roll=args.roll
        )
    )

    print("Setting new transform:", new_transform)
    ego_vehicle.set_transform(new_transform)

    # If running in synchronous mode, tick once
    settings = world.get_settings()
    if settings.synchronous_mode:
        print("World in synchronous mode -> ticking once")
        world.tick()

    print("Ego vehicle reset complete.")
    print("New transform:", ego_vehicle.get_transform())


if __name__ == '__main__':
    main()