cmake_minimum_required(VERSION 2.8.3)
project(robotiq_85_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roslaunch)

## Declare a catkin package
catkin_package()

roslaunch_add_file_check(launch)

## Install ## 
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


