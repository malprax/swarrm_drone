#!/bin/bash

# Jalankan fix_environment.sh otomatis
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/fix_environment.sh"

echo "Starting Swarm Patrol 2D Simulation..."

# Launch patrol
roslaunch swarm_patrol patrol_2d.launch &

sleep 5

# Launch RViz
roslaunch swarm_patrol launch_rviz.launch &

sleep 3

# Launch rqt_graph
roslaunch swarm_patrol launch_rqt_graph.launch &

echo "Swarm Patrol 2D Simulator Launched Successfully!"