cmake_minimum_required(VERSION 3.0.2)
project(rover_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs std_srvs)

add_message_files(
  DIRECTORY msg
  FILES
  roverBmsStatus.msg
  roverRCStatus.msg
  roverGoalStatus.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
