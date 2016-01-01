cmake_minimum_required(VERSION 2.8.3)
project(uav_debug)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf)

find_package(PCL REQUIRED)

catkin_package()

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(uav_debug src/uav_debug.cpp)
target_link_libraries(uav_debug ${catkin_LIBRARIES})

install(
    TARGETS uav_debug
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
