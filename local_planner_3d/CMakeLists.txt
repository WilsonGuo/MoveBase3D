cmake_minimum_required(VERSION 3.0.2)
project(local_planner_3d)

## 编译选项
add_compile_options(-std=c++11)

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  visualization_msgs
  rover_msgs
)
find_package(Eigen3 REQUIRED)

## 声明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs tf visualization_msgs rover_msgs
)

## 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 安装节点脚本和启动文件
catkin_install_python(PROGRAMS
  scripts/local_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
