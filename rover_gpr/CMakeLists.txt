cmake_minimum_required(VERSION 3.8)
project(rover_gpr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(memory REQUIRED)
# find_package(fstream REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY config include launch  DESTINATION share/${PROJECT_NAME})

add_executable(gpr_path src/gpr_path.cpp)

include_directories(include)  # 包含include/rover_gpr目录

ament_target_dependencies(
  gpr_path
  "rclcpp"
  "std_msgs"
  "Eigen3"
)

install(TARGETS
  gpr_path
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
