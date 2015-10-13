cmake_minimum_required(VERSION 2.8.3)
project(uav_servo_node)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(
    uav_servo_node 
    src/Dynamixel.cpp 
    src/ErrorMessage.cpp 
    src/SerialDevice.cpp 
    src/servo_node.cpp)
target_link_libraries(uav_servo_node ${catkin_LIBRARIES})
