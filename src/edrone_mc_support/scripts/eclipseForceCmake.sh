#! /bin/bash

COM=catkin build $1 --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
echo $COM
catkin build $1 --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
