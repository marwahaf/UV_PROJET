#!/usr/bin/env sh

sudo apt install -y
    git \
    ros-noetic-ecl-exceptions \
    ros-noetic-ecl-threads \
    ros-noetic-ecl-geometry \
    ros-noetic-ecl-streams \
    ros-noetic-kobuki-* \
    ros-noetic-depthimage-to-laserscan \
    ros-noetic-joy \
    ros-noetic-urg-node

# not there in develter
sudo apt install -y
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-map-server \
    ros-noetic-gmapping
