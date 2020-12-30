#!/bin/bash
gnome-terminal --tab -- /bin/bash -c 'source ./devel/setup.sh; roslaunch i214_description spawn.launch'
sleep 3

gnome-terminal --tab -- /bin/bash -c 'source ./devel/setup.sh; roslaunch i214_description rviz.launch'
gnome-terminal --tab -- /bin/bash -c 'source ./devel/setup.sh; rosrun gazebo_ros gazebo'