#!/bin/bash
gnome-terminal --tab -- /bin/bash -c 'source ./devel/setup.sh; rosrun motion_plan vfh.py'

gnome-terminal --tab -- /bin/bash -c 'source ./devel/setup.sh; rosrun motion_plan spiral.py'
gnome-terminal --tab -- /bin/bash -c 'source ./devel/setup.sh; rosrun motion_plan navigation.py'