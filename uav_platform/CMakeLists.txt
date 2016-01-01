cmake_minimum_required(VERSION 2.8.3)
project(uav_platform)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp tf uav_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(pose_filter src/pose_filter.cpp)
target_link_libraries(pose_filter ${catkin_LIBRARIES})
add_dependencies(pose_filter ${catkin_EXPORTED_TARGETS})

add_executable(align_mode src/align_mode.cpp)
target_link_libraries(align_mode ${catkin_LIBRARIES})
add_dependencies(align_mode ${catkin_EXPORTED_TARGETS})

add_executable(nav_markers src/nav_markers.cpp)
target_link_libraries(nav_markers ${catkin_LIBRARIES})
add_dependencies(nav_markers ${catkin_EXPORTED_TARGETS})

install(
    TARGETS pose_filter align_mode nav_markers
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
