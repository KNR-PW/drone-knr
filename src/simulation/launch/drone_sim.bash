#!/bin/bash

gnome-terminal -- ros2 launch simulation drone_sim_launch.py
gnome-terminal -- ign gazebo -v 1 -r tree_of_life.sdf

cd ~/ardupilot
gnome-terminal -- sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console