cmake_minimum_required(VERSION 2.8.3)
project(robot_state_publisher)

set(CMAKE_BUILD_TYPE Release)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS
        kdl_parser 
        roscpp 
        sensor_msgs 
        tf 
        tf_conversions)

find_package(orocos_kdl REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES robot_state_publisher
    CATKIN_DEPENDS roscpp sensor_msgs tf
    DEPENDS Boost orocos_kdl)

# note: must come before catkin_INCLUDE_DIRS so the robot_state_publisher.h from
# this package is found before the one available in /opt/ros/<distro>
include_directories(include) 

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_library(
    robot_state_publisher 
    src/robot_state_publisher.cpp 
    src/treefksolverposfull_recursive.cpp)
target_link_libraries(
    robot_state_publisher 
    ${catkin_LIBRARIES} 
    ${orocos_kdl_LIBRARIES})

add_executable(state_publisher src/joint_state_listener.cpp)
target_link_libraries(
    state_publisher 
    ${catkin_LIBRARIES} 
    ${orocos_kdl_LIBRARIES} 
    robot_state_publisher)
