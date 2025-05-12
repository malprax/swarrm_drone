#!/bin/bash

# Cari semua container aktif yang berbasis image ROS Noetic
container_id=$(docker ps -q --filter "ancestor=osrf/ros:noetic-desktop-full")

if [ -n "$container_id" ]; then
  echo "Stopping Swarm Patrol Docker Container..."
  docker stop $container_id
  echo "Container stopped!"
else
  echo "No running Swarm Patrol container found."
fi
