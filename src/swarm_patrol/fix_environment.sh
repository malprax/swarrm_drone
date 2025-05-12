#!/bin/bash

echo "Fixing ROS environment variables..."

# Tambahkan folder binary ROS ke PATH
export PATH=$PATH:/opt/ros/noetic/bin

# Tambahkan folder Python libraries ROS ke PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages

# Tambahkan folder shared libraries ROS ke LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib

# Source core ROS Noetic
source /opt/ros/noetic/setup.bash

# Source workspace swarm_ws
source ~/swarm_ws/devel/setup.bash

echo "✅ Environment fixed! You can now use roslaunch, rospack, etc."
