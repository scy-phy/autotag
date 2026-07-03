# Dataset Collection - By setting up the physical environment and attacks/benign scenario

Once Carla --> Bridge --> Autoware are spun up successfully as described in `run.md`, we have additional configuration files and python scripts to support collection of different autoware topics into jsons along with image and lidar data from autoware sensors, which could be used e.g. for Machine Learning.

All scripts for dataset collection must be run from the bridge container, which can be found via:
- Find the docker container of the bridge: e.g. run `docker ps`
- Exec into the container: `docker exec -it [cont_name] bash`
- Then inside the container, run the scripts, e.g.: `generate_weather.py`, `generate_traffic.py`

Dataset is collected per topic (topics mentioned in the config file) framewise. This means that for each topic, a json file is generated which has rows of data where each row correspond to values published by that topic at a certain timeframe. Each column represents values of different fields published by that topic.

For image and lidar data, different folders are created for different sensors (front camera, lidar, etc.) and its respective images or lidar cloud points are stored framewise. 

The timeframe acts as a key into this extensive dataset, for the rows of the json files as well as the name of the image and lidar files.

### Configuration Files

There are 2 configuration files to be created (defaults provided). One to describe the physical environment to be applied and ros topics to collect for the dataset and one to configure attacks if collecting datasets for attack scenarios. If dataset being collected is for benign scenarios, the attack configuration file is not required.

### Setting up Dependencies in Bridge Terminals
Sourcing some bash scripts are required to have autoware and ros dependencies installed in the bridge terminals.

Run the below script once from any one of the terminals that have the bridge container running:

```
cd src/carla_autoware_bridge/utils/dataset_collection
./build_messages.sh
```

Source the below script before running the dataset collection scripts (weather, traffic, route, etc.) in every terminal that has the bridge container running:

```
source install/setup.bash
```

### Setting up the Physical Environment
The `generate_weather.py` and `generate_traffic.py` scripts are used to setup different weather and traffic conditions in Carla before we begin dataset collection.

From a new terminal, run the following commands from the bridge container:
```
cd src/carla_autoware_bridge/utils/dataset_collection
python3 generate_weather.py --config default_config.yaml
```

From a new terminal, run the following commands from the bridge container:
```
cd src/carla_autoware_bridge/utils/dataset_collection
python3 generate_traffic.py --config default_config.yaml
```

- `--config` name of the config file. Default: 'default_config.yaml'


### Setting up the Route for the Ego Vehicle and Triggering Dataset Collection

`ros_message_dataset.py` triggers dataset collection into the folder mentioned in the config file after it discovers all the required topics.

`set_route_engage_vehicle.py` sets route based on the config file and engages the ego vehicle into autonomous mode to start moving.

From a new terminal run the following commands from the bridge container:
```
cd src/carla_autoware_bridge/utils/dataset_collection
python3 ros_message_dataset.py --config default_config.yaml
```

From a new terminal run the following commands from the bridge container:
```
cd src/carla_autoware_bridge/utils/dataset_collection
python3 set_route_vehicle_engage --config default_config.yaml
```

- `--config` name of the config file. Default: 'default_config.yaml'

If you'd like to trigger an attack at any point during the vehicle's journey through the route,  attack scripts can be run as described above.


### Triggering Attacks
There are two different implementations of injecting attacks:

- Sensor-level: One implementation directly modifies the sensor data in the bridge, i.e. the data is manipulated before the sensor data coming from Carla are transferred into ROS messages to be used by Autoware. We show this at the example of the lidar data, where we implement one specific attack aiming at hiding a front vehicle. However the implementation allows to easily integrate different manipulations. Details can be found in `docs/attacks/lidar_attack.md` 
- AV-level: The second implementation works on the AV-level by modifying specific topics in Autoware. We implement this at the example of the perception stack by modifying bounding boxes. Thereby different types of attacks can be easily simulated. E.g. the implementation allows to drop bounding boxes to simulate hiding attacks, shift positions of bounding boxes, etc.. Details can be found in `docs/attacks/bounding_box_modifier.md`

Details of how to run the attacks can be found in the respective attack documentation. In general attacks could be run at any point. For metadata collection ensure to adapt the general config file with the attack information (path to config, optionally name and description). 

The dataset collection labels the attacked frames automatically to ensure ground truth information. 


### Implementing Other Attacks

#TODO