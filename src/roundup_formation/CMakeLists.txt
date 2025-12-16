cmake_minimum_required(VERSION 3.0.2)
project(roundup_formation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roundup_optimizer
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs pcl_ros pcl_conversions
  DEPENDS EIGEN3 CERES PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Library
add_library(roundup_optimizer
  src/roundup_optimizer.cpp
)
target_link_libraries(roundup_optimizer
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Node
add_executable(roundup_formation_node src/roundup_formation_node.cpp)
target_link_libraries(roundup_formation_node
  roundup_optimizer
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS roundup_optimizer roundup_formation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)