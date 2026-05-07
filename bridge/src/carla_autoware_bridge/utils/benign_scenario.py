# spawn_car.py

import carla
import argparse
import time

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=1403)
    parser.add_argument("--spawn-index", type=int, default=75)
    parser.add_argument("--bp-number", type=int, default=1)
    args = parser.parse_args()

    client = carla.Client("localhost", args.port)
    client.set_timeout(20.0)

    vehicles_list = []

    try:
        world = client.get_world()
        traffic_manager = client.get_trafficmanager(8000)

        settings = world.get_settings()
        synchronous_master = False

        traffic_manager.set_synchronous_mode(True)

        if not settings.synchronous_mode:
            synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

        #Get vehicle blue print
        blueprint_library = world.get_blueprint_library()
        vehicle_blueprints = sorted(
            blueprint_library.filter("vehicle"),
            key=lambda bp: bp.id
        )

        if args.bp_number < 0 or args.bp_number >= len(vehicle_blueprints):
            print("Invalid blueprint index")
            return

        blueprint = vehicle_blueprints[args.bp_number]

        if blueprint.has_attribute('color'):
            blueprint.set_attribute(
                'color',
                blueprint.get_attribute('color').recommended_values[0]
            )

        blueprint.set_attribute('role_name', 'autopilot')

        #Spawn point
        spawn_points = world.get_map().get_spawn_points()

        if args.spawn_index < 0 or args.spawn_index >= len(spawn_points):
            print("Invalid spawn index")
            return

        base_transform = spawn_points[args.spawn_index]

        #Some offset to actual spawn point
        transform = carla.Transform(
            carla.Location(
                x=base_transform.location.x,
                y=base_transform.location.y + 0.5,
                z=base_transform.location.z
            ),
            base_transform.rotation
        )

        #Spawn the actor
        SpawnActor = carla.command.SpawnActor

        batch = [SpawnActor(blueprint, transform)]

        responses = client.apply_batch_sync(batch, synchronous_master)

        for response in responses:
            if response.error:
                print("Spawn error:", response.error)
            else:
                vehicles_list.append(response.actor_id)

        print(f"Spawned {len(vehicles_list)} vehicle(s)")

        #Keep alive until script closed
        print("Press Ctrl+C to exit")

        while True:
            if synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:
        # Cleanup
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print(f"Destroying {len(vehicles_list)} vehicles")
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        time.sleep(0.5)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("Done.")