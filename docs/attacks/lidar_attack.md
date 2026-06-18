# Lidar Attack

Reproduction of the paper "Can We Use Arbitrary Objects to Attack LiDAR Perception in Autonomous Driving?". 

The attack is approximated by manipulating the lidar data in the bridge between Autoware and Carla. The attack proposes to put lidar points at specific locations around a target object (car) to "hide" it from being detected. In the original paper this was realized by placing pieces of cardboard via a drone at the specific locations. We approximate this by adding lidar points in a square at the specific locations. 


## Running the attack

To run the attack, we modified the bridge code. Run Carla and Autoware normally. For the bridge docker do the following:

Start the docker including the attack code:
```
docker run -it -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///cyclonedds.xml --volume ~/cyclonedds.xml:/cyclonedds.xml -v ~/cypherAV/cypher-av/bridge/src/carla_autoware_bridge:/tum/src/carla_autoware_bridge -v ~/cypherAV/cypher-av/bridge/src/ros-bridge/attack/carla_ros_bridge:/tum/src/carla/ros-bridge/carla_ros_bridge/src/carla_ros_bridge -v ~/cypherAV/cypher-av/bridge/src/lidar_attack_msgs:/tum/src/lidar_attack_msgs   --network host tumgeka/carla-autoware-bridge:latest
```

Then inside the docker run the following commands:
```
colcon build --packages-select autoware_auto_perception_msgs
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
Exec into the bridge container in another terminal and run: `python3 benign_scenario.py`

Publish the attack config message:

Start another bridge container with the same command as above.
In the container run:

#TODO: config

`python3 run_attack.py`



## Finding the vulnerable locations