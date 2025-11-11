#!/bin/bash

xhost +local:root

# start gazebo simulation
docker compose up -d world
sleep 2

# start ground controls station
docker compose up -d gcs
sleep 2

# Start flight stack
docker compose up -d px4
sleep 2

# Start state machine viewer
docker compose up -d fsm
sleep 2
xdg-open http://localhost:5000

# Start a development container
docker compose up -d dev
sleep 2
docker compose exec -it dev bash