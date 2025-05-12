# Swarm Patrol 2D Simulator (MacOS + Docker)

## Install Requirements
- Homebrew
- XQuartz
- Docker Desktop

## Setup XQuartz
- Allow connections from network clients
- Restart XQuartz
- Run `xhost +`

## Run Docker Container
Use mounted volume to keep workspace:

```bash
./start_docker_swarm.sh



command run docker 
docker run -it --rm --name ros_noetic_v2 -e DISPLAY=host.docker.internal:0 ros:noetic