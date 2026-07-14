#!/usr/bin/env python

import glob
import os
import sys
import argparse
import yaml

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
    argparser.add_argument('--config', default='default_config.yaml', help='Path to config.yaml')
    args = argparser.parse_args()

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    scenario = config.get('scenario', {})
    host = scenario.get('host', '127.0.0.1')
    port = scenario.get('port', 2000)
    weather_env = scenario.get('weather', {}).get('environment', 1)

    client = carla.Client(host, port)
    client.set_timeout(2.0)
    world = client.get_world()

    # ==========================================
    # Weather Selection
    # 1 = Clear Day
    # 2 = Rainy Day
    # 3 = Clear Night
    # 4 = Rainy Night
    # ==========================================

    weather = carla.WeatherParameters()

    if weather_env == 1:
        print("Setting weather: Clear Day")
        weather.sun_altitude_angle = 75.0
        weather.sun_azimuth_angle = 0.0
        weather.cloudiness = 0.0
        weather.precipitation = 0.0
        weather.precipitation_deposits = 0.0
        weather.wind_intensity = 0.0
        weather.fog_density = 0.0
        weather.wetness = 0.0

    elif weather_env == 2:
        print("Setting weather: Rainy Day")
        weather.sun_altitude_angle = 50.0
        weather.sun_azimuth_angle = 0.0
        weather.cloudiness = 80.0
        weather.precipitation = 80.0
        weather.precipitation_deposits = 80.0
        weather.wind_intensity = 40.0
        weather.fog_density = 10.0
        weather.wetness = 100.0

    elif weather_env == 3:
        print("Setting weather: Clear Night")
        weather.sun_altitude_angle = -90.0 
        weather.sun_azimuth_angle = 0.0
        weather.cloudiness = 0.0
        weather.precipitation = 0.0
        weather.precipitation_deposits = 0.0
        weather.wind_intensity = 0.0
        weather.fog_density = 0.0
        weather.wetness = 0.0

    elif weather_env == 4:
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
        print("Invalid Environment selected in config.yaml. Please choose 1, 2, 3, or 4.")
        sys.exit(1)

    world.set_weather(weather)
    print("Weather successfully updated.")

if __name__ == '__main__':
    main()
