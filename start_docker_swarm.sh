#!/bin/bash

# Lokasi folder lokal dan dalam container
LOCAL_DIR="$HOME/docker_shared/swarm_ws"
CONTAINER_DIR="/root/swarm_ws"

# Pastikan folder lokal tersedia
# mkdir -p "$LOCAL_DIR/src"

# Set DISPLAY untuk Docker agar bisa akses XQuartz
# export DISPLAY=host.docker.internal:0

# Izinkan akses X server dari localhost (hanya perlu 1x di terminal host)
xhost + 127.0.0.1 

# Hentikan container jika sudah ada
if [ "$(docker ps -aq -f name=swarm_patrol_container)" ]; then
    echo "⚠️  Container sudah ada, menghentikan..."
    docker stop swarm_patrol_container >/dev/null
    docker rm swarm_patrol_container >/dev/null
    echo "🗑️  Container lama dihapus."
fi

# Jalankan container Docker
# --env="DISPLAY=$DISPLAY" \
# --name swarm_patrol_container \
#   osrf/ros:noetic-desktop-full
# ros:noetic
docker run -it -d \
  --platform linux/amd64 \
  --env="DISPLAY=host.docker.internal:0" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority" \
  --volume="$LOCAL_DIR:$CONTAINER_DIR" \
  --privileged \
  --name swarm_patrol_container \
    osrf/ros:noetic-desktop-full

echo "✅ Container 'swarm_patrol_container' sudah dijalankan."
echo "👉 Untuk masuk: docker exec -it swarm_patrol_container /bin/bash"