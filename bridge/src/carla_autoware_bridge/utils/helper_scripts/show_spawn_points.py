# publish_spawn_points.py

import carla
import argparse
import time

DRAW_TIME = 100.0  # seconds debug points stay visible

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=1403)
    args = parser.parse_args()

    client = carla.Client("localhost", args.port)
    client.set_timeout(20.0)

    world = client.get_world()
    original_settings = world.get_settings()

    try:
        spawn_points = world.get_map().get_spawn_points()

        print(f"Found {len(spawn_points)} spawn points:\n")

        for i, sp in enumerate(spawn_points):
            loc = sp.location

            # Print info
            print(f"[{i}] x={loc.x:.2f}, y={loc.y:.2f}, z={loc.z:.2f}")

            # Draw debug point
            world.debug.draw_string(
                loc,
                str(i),
                draw_shadow=False,
                color=carla.Color(255, 0, 0),
                life_time=DRAW_TIME,
                persistent_lines=False
            )

        print(f"\nSpawn points drawn for {DRAW_TIME} seconds...")

        # Keep script alive so drawings persist
        time.sleep(DRAW_TIME)

    finally:
        # Restore settings to avoid breaking CARLA
        world.apply_settings(original_settings)
        print("World settings restored. Exiting cleanly.")

if __name__ == "__main__":
    main()