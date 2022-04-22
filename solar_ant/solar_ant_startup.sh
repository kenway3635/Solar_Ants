#!/bin/bash
cd /home/ubuntu/catkin_ws
catkin_make
source /opt/ros/melodic/setup.bash
. ~/catkin_ws/devel/setup.bash
sudo chmod 777 /dev/ttyACM0
roslaunch solar_ant solar_ant_startup.launch
rosrun rosserial_python serial_node.py /dev/ttyACM0


