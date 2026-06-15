#!/usr/bin/env bash

echo "Updating package lists..."
apt-get update

echo "Importing extra ROS repositories..."
cd /tum/src/carla_autoware_bridge/extra_messages
vcs import /tum/src < extra_messages.repos

echo "Installing ROS dependencies..."
cd /tum
rosdep install --from-paths src --ignore-src -r -y

echo "Building workspace..."
colcon build --symlink-install --packages-skip-build-finished

echo "Sourcing workspace..."
source install/setup.bash

echo "Done."
