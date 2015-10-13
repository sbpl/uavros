cmake_minimum_required(VERSION 2.8.3)
project(uav_state_publisher)

set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS 
        geometry_msgs 
        nav_msgs 
        pcl_conversions
        sensor_msgs 
        tf 
        uav_msgs)

catkin_package()

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(uav_state_publisher src/uav_state_publisher.cpp)
target_link_libraries(uav_state_publisher ${PCL_LIBRARIES} ${catkin_LIBRARIES})
