#!/bin/bash

# Folder yang akan di-mount dari Mac ke Docker
LOCAL_DIR="$HOME/docker_shared/swarm_ws"
CONTAINER_DIR="/root/swarm_ws"

# Pastikan folder lokal ada
mkdir -p "$LOCAL_DIR/src"

# Jalankan Docker container
docker run -it -d \
    --platform linux/amd64 \
    --env="DISPLAY=host.docker.internal:0" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$LOCAL_DIR:$CONTAINER_DIR" \
    --privileged \
    --name swarm_patrol_container \
    osrf/ros:noetic-desktop-full

# Info buat user
echo "Container 'swarm_patrol_container' sudah jalan."
echo "Gunakan perintah berikut untuk masuk ke dalam container:"
echo "docker exec -it swarm_patrol_container /bin/bash"
