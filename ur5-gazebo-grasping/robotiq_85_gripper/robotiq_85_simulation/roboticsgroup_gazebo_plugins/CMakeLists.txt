cmake_minimum_required(VERSION 2.8.3)
project(roboticsgroup_gazebo_plugins)

macro(use_cxx11)
    if (CMAKE_VERSION VERSION_LESS "3.1")
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
        endif ()
    else ()
        set (CMAKE_CXX_STANDARD 11)
    endif ()
endmacro(use_cxx11)

use_cxx11()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  control_toolbox
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    control_toolbox
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

add_library(roboticsgroup_gazebo_mimic_joint_plugin src/mimic_joint_plugin.cpp)
target_link_libraries(roboticsgroup_gazebo_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(roboticsgroup_gazebo_disable_link_plugin src/disable_link_plugin.cpp)
target_link_libraries(roboticsgroup_gazebo_disable_link_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
