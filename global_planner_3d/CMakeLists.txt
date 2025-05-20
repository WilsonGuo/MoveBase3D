cmake_minimum_required(VERSION 3.8)
project(global_planner_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenMP)

install(DIRECTORY config include  DESTINATION share/${PROJECT_NAME})

add_executable(global_planning_node src/global_planning_node.cpp
                                    src/execution_classes.cpp
                                    src/execution_planner.cpp
                                    src/execution_vis.cpp)
add_executable(global_planning_obs src/global_planning_obs.cpp
                                    src/execution_classes.cpp
                                    src/execution_planner.cpp
                                    src/execution_vis.cpp)
                                    

include_directories(include)

ament_target_dependencies(
  global_planning_node
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "visualization_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "rclcpp"
  "rover_msgs"
  "PCL"
  "pcl_conversions"
  "Eigen3"
)

ament_target_dependencies(
  global_planning_obs
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "visualization_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "rclcpp"
  "rover_msgs"
  "PCL"
  "pcl_conversions"
  "Eigen3"
)

install(TARGETS
  global_planning_node
  global_planning_obs
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

if(OpenMP_CXX_FOUND)
  target_link_libraries(global_planning_node OpenMP::OpenMP_CXX)
  target_link_libraries(global_planning_obs OpenMP::OpenMP_CXX)
endif()
