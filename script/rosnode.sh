#/!bin/bash
gnome-terminal -- rosrun aerointer takeoff.py &&
sleep 10
gnome-terminal -- rosrun aerointer arucocam_node.py &&
sleep 5
gnome-terminal -- rosrun aerointer obstacle_avoidance.py
