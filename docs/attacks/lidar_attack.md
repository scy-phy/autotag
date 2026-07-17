# Lidar Attack

AutoTAG implements sensor attacks using a modular attack framework integrated into the bridge between CARLA and Autoware. The bridge receives the original sensor data from CARLA, optionally applies an attack, and then publishes the manipulated data as ROS messages consumed by Autoware.

Currently, this framework is implemented for the front LiDAR sensor, but the same mechanism can be applied to other sensors by modifying their corresponding sensor implementation (e.g., `camera.py`, `imu.py`, etc.) in the same way as `lidar.py`.

Below we first describe how to run the concrete attack we implemented and then how to integrate new (lidar) attacks.

## Lidar Patch Attack

Reproduction of the paper "Can We Use Arbitrary Objects to Attack LiDAR Perception in Autonomous Driving?". 

The attack is approximated by manipulating the lidar data in the bridge between Autoware and Carla. The attack proposes to put lidar points at specific locations around a target object (car) to "hide" it from being detected. In the original paper this was realized by placing pieces of cardboard via a drone at the specific locations. We approximate this by adding lidar points in a square at the specific locations. 


### Running the attack

To run the attack, we modified the bridge code. Run Carla and Autoware normally. For the bridge docker do the following:

Start the docker including the attack code:
```
docker run -it -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///cyclonedds.xml --volume ~/cyclonedds.xml:/cyclonedds.xml -v $(pwd)/bridge/src/carla_autoware_bridge:/tum/src/carla_autoware_bridge -v $(pwd)/bridge/src/ros-bridge/attack/carla_ros_bridge:/tum/src/carla/ros-bridge/carla_ros_bridge/src/carla_ros_bridge -v $(pwd)/bridge/src/lidar_attack_msgs:/tum/src/lidar_attack_msgs   --network host tumgeka/carla-autoware-bridge:latest
```

Then inside the docker build the autoware messages (as described in `run.md`) and then run the following commands:
```
cd /tum
colcon build --packages-select lidar_attack_msgs
source install/setup.bash
```
Then start the bridge as normally:
```
ros2 launch carla_autoware_bridge carla_aw_bridge.launch.py port:=1403 town:=Town10HD timeout:=60
```

Then to start the attack we need to spawn a target vehicle and publish the configuration where we want to place the patches.
Note: the attack assumes that the target vehicle is in front of the ego vehicle. The patch position is computed relative to the target vehicle. 

Create an target vehicle:
- Optionally: To show possible spawn points in carla run:
```
cd src/carla_autoware_bridge/utils/helper_scripts
python3 show_spawn_points.py
```
- Configure the spawn point where the target should be spawned via the central configuration file under `scenario`:
```
  target_vehicle: 
    spawn_index: 75 #spawn point
    bp_number: 1 #to choose different types of vehicles
```
- Exec into the bridge container in another terminal and run: 
```
cd src/carla_autoware_bridge/utils/dataset_collection
python3 target_scenario.py
```

Publish the attack config message:
- Configure the patch locations and exact front region where the attack should be applied via the config file: `bridge/src/carla_autoware_bridge/utils/sensorAttack/attack_config.yaml`

- Exec into the bridge container from another terminal (make sure the lidar attack messages (see above) are built and everthing is sourced):
- In the container run:
```
python run_attack.py --config attack_config.yaml
```
`--config` is optional. By default: `attack_config.yaml`

- To disable the attack again, run:
```
python run_attack.py --disable
```
- Note: If the attack is not disabled properly, it stays active even if the attack script is not running anymore!



## Integrating new attacks

We now describe how to integrate new attacks in a similar way to the above.

### General Architecture

The central component is the `AttackManager`, which is responsible for managing attack instances during runtime.

For every incoming LiDAR frame, the attack manager:

1. receives the latest attack configuration from the ROS configuration topic,
2. creates or updates the selected attack implementation,
3. forwards the incoming point cloud to the attack implementation,
4. returns the modified point cloud to the bridge.

Only one attack is active at a time. Attacks can be enabled, disabled, or reconfigured during runtime without restarting the bridge.

The attack manager itself contains no attack logic. It simply dispatches incoming sensor data to the selected attack implementation.

All of the low-level code for the attack is in the following files/folders:
- `bridge/src/ros-bridge/attack/carla_ros_bridge/lidar.py`
- `bridge/src/ros-bridge/attack/carla_ros_bridge/attack`

### Implementing a New Attack

All LiDAR attacks inherit from `BaseLidarAttack` (see: `base_lidar_attack.py`). 

- To create a new attack create a new class that inherits from this class. For an example see: `arbitrary_object_patch.py`
- The new attack class should implement the following methods: `apply(lidar_data)`, `update_parameters(parameters)`, `disable_attack()`
    - `apply(lidar_data)` performs the actual manipulation of the lidar data and returns the modified pointcloud. Note: In order for the attack status message to work properly (necessary for attack labeling during dataset collection) the `attack_applied` status returned by the function needs to properly reflect whether the attack is applied in the current frame or not. 
    - `update_parameters(parameters)` updates the attack configuration whenever a new configuration message is received.
    - `disable_attack()` resets any internal state when the attack is disabled.

- After implementing the attack class, register it inside `attack_manager.py`

Attack activation is controlled through a ROS message. This message has fields for attack type, attack class and any optional parameters that are needed to configure the specific attack. 

- To create such a message, modify the `attack_config.yaml` in `bridge/src/carla_autoware_bridge/utils/sensorAttack/`:
    - `attack_run_info` is mandatory and there the `attack_type` must match what is specified in the attack manager
    - any other parameters can be added
- Update the `run_attack.py` in `bridge/src/carla_autoware_bridge/utils/sensorAttack/` to correctly read the custom parameters

The attack can then be run in the same way as described above, i.e. by running `python3 run_attack.py` and optionally running any specific target scenario.


The constructor receives the parameter dictionary from the ROS configuration message and can initialize any attack-specific state.

### Extending the Framework to Other Sensors

The current implementation supports LiDAR attacks only.

To support another sensor (e.g., cameras or radar), the corresponding sensor implementation needs to be modified similarly to `lidar.py`. In particular, the sensor implementation should:

1. create an `AttackManager` for the sensor,
2. subscribe to the attack configuration topic,
3. forward the sensor data to the attack manager before publishing,
4. publish the corresponding attack status message.

The individual attack implementations remain independent of the bridge and only implement the sensor manipulation itself.

