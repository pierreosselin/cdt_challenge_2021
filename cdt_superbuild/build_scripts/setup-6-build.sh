#!/bin/bash


WORKING_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../../ && pwd )"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/../../../ && pwd )"

cd ${WORKING_DIR}/catkin_ws

source /opt/ros/melodic/setup.bash

echo "Building catkin workspace"
catkin build --continue-on-failure --cmake-args -DCMAKE_CXX_COMPILER="/usr/bin/g++" -DCMAKE_C_COMPILER="/usr/bin/gcc"
if [ $? -ne 0 ]; then
    echo "Build of catkin workspace failed"
    exit 7
fi
