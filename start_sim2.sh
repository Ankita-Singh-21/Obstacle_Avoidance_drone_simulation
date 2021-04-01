#!/bin/bash

gnome-terminal -- roslaunch interiit21 interiit_world2.launch &&
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris



