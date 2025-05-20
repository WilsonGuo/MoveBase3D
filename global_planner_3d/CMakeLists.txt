cmake_minimum_required(VERSION 3.0.2)
project(global_planner_3d)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rviz_visual_tools
  std_msgs
  tf
  rover_msgs
)

find_package(PCL REQUIRED)

catkin_package(
INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable(global_planning_node
    src/global_planning_node.cpp
    src/execution_planner.cpp
    src/execution_classes.cpp
    src/execution_vis.cpp)

target_link_libraries(global_planning_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

add_executable(global_planning_obs_node
    src/global_planning_obs.cpp
    src/execution_planner.cpp
    src/execution_classes.cpp
    src/execution_vis.cpp)

target_link_libraries(global_planning_obs_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${tf_LIBRARIES}
)




