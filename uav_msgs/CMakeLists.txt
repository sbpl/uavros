cmake_minimum_required(VERSION 2.8.3)
project(uav_msgs)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_message_files(
    DIRECTORY msg
    FILES
        BehaviorPath.msg 
        ControllerCommand.msg 
        FlightModeRequest.msg 
        FlightModeStatus.msg 
        PoseAction.msg 
        camera_msg.msg 
        mode_msg.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)
