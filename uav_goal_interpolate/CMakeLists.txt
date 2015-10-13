cmake_minimum_required(VERSION 2.8.3)
project(uav_goal_interpolate)

set(CMAKE_BUILD_TYPE Release)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp uav_msgs)

catkin_package()

add_executable(uav_goal_interpolate src/uav_goal_interpolate.cpp)
target_link_libraries(uav_goal_interpolate ${Boost_LIBRARIES} ${catkin_LIBRARIES})
