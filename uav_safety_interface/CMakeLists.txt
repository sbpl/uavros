cmake_minimum_required(VERSION 2.8.3)
project(uav_safety_interface)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS 
        geometry_msgs 
        laser_geometry 
        sensor_msgs 
        tf 
        uav_msgs 
        visualization_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(uavSafetyInterface src/SafetyInterface.cpp)
target_link_libraries(uavSafetyInterface ${catkin_LIBRARIES})
add_dependencies(uavSafetyInterface ${catkin_EXPORTED_TARGETS})

add_executable(testController src/TestController.cpp)
target_link_libraries(testController ${catkin_LIBRARIES})
add_dependencies(testController ${catkin_EXPORTED_TARGETS})
