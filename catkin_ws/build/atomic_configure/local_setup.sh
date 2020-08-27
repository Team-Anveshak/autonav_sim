#!/usr/bin/env sh
# generated from catkin/cmake/template/local_setup.sh.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
<<<<<<< HEAD
: ${_CATKIN_SETUP_DIR:=/home/kevin/autonav_sim/catkin_ws/devel}
=======
: ${_CATKIN_SETUP_DIR:=/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/devel}
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
CATKIN_SETUP_UTIL_ARGS="--extend --local"
. "$_CATKIN_SETUP_DIR/setup.sh"
unset CATKIN_SETUP_UTIL_ARGS
