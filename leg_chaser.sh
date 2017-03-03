#!/bin/sh

roslaunch openni_launch openni.launch
rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth/image_raw
rosrun leg_chaser leg_chaser_node
rosrun rviz rviz

exit 0
