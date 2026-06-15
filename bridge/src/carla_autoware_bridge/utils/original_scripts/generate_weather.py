#!/usr/bin/env python

import glob
import os
import sys
import argparse

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def main():
    argparser = argparse.ArgumentParser(description='CARLA Static Weather Setter')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-e', '--environment', metavar='E', default=1, type=int, help='Environment/Weather: 1=ClearDay, 2=RainyDay, 3=ClearNight, 4=RainyNight')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    # ==========================================
    # Environment Selection
    # 1 = Clear Day
    # 2 = Rainy Day
    # 3 = Clear Night
    # 4 = Rainy Night
    # ==========================================
    scenario = args.environment

    weather = carla.WeatherParameters()

    if scenario == 1:
        print("Setting weather: Clear Day")
        weather.sun_altitude_angle = 75.0
        weather.sun_azimuth_angle = 0.0
        weather.cloudiness = 0.0
        weather.precipitation = 0.0
        weather.precipitation_deposits = 0.0
        weather.wind_intensity = 0.0
        weather.fog_density = 0.0
        weather.wetness = 0.0

    elif scenario == 2:
        print("Setting weather: Rainy Day")
        weather.sun_altitude_angle = 50.0
        weather.sun_azimuth_angle = 0.0
        weather.cloudiness = 80.0
        weather.precipitation = 80.0
        weather.precipitation_deposits = 80.0
        weather.wind_intensity = 40.0
        weather.fog_density = 10.0
        weather.wetness = 100.0

    elif scenario == 3:
        print("Setting weather: Clear Night")
        # A negative altitude angle puts the sun below the horizon
        weather.sun_altitude_angle = -90.0 
        weather.sun_azimuth_angle = 0.0
        weather.cloudiness = 0.0
        weather.precipitation = 0.0
        weather.precipitation_deposits = 0.0
        weather.wind_intensity = 0.0
        weather.fog_density = 0.0
        weather.wetness = 0.0

    elif scenario == 4:
        print("Setting weather: Rainy Night")
        weather.sun_altitude_angle = -90.0
        weather.sun_azimuth_angle = 0.0
        weather.cloudiness = 80.0
        weather.precipitation = 80.0
        weather.precipitation_deposits = 80.0
        weather.wind_intensity = 40.0
        weather.fog_density = 10.0
        weather.wetness = 100.0
    
    else:
        print("Invalid Environment selected. Please choose 1, 2, 3, or 4.")
        sys.exit(1)

    world.set_weather(weather)
    print("Weather successfully updated.")

if __name__ == '__main__':
    main()
