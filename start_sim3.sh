#!/bin/bash

gnome-terminal -- roslaunch interiit21 empty_world.launch &&
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris



