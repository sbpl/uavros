cmake_minimum_required(VERSION 2.8.3)
project(uav_camera)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
    COMPONENTS
        cv_bridge 
        dynamic_reconfigure 
        image_transport
        roscpp 
        uav_msgs)

generate_dynamic_reconfigure_options(cfg/threshold.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(video_publisher src/video_publisher.cpp)
target_link_libraries(video_publisher ${catkin_LIBRARIES})
add_dependencies(video_publisher ${uav_camera_EXPORTED_TARGETS})
