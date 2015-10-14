cmake_minimum_required(VERSION 2.8.3)
project(uav_set_goal)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS 
        geometry_msgs 
        interactive_markers
        uav_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(uav_set_goal src/uav_set_goal.cpp)
target_link_libraries(uav_set_goal ${catkin_LIBRARIES})
add_dependencies(uav_set_goal ${catkin_EXPORTED_TARGETS})
