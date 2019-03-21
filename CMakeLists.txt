cmake_minimum_required(VERSION 2.8.3)
project(lslidar_3d_slam)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -o3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

set(G2O_LIBS
  g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d
)

catkin_package(
)

include_directories(
  "/usr/include/eigen3"
  src
  include/lslidar_3d_slam
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
)

add_executable(lslidar_3d_slam
  src/floor_filter.cpp
  src/map_builder.cpp
  src/lidar_slam_3d_ros.cpp
  src/node.cpp
)

target_link_libraries(lslidar_3d_slam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${G2O_LIBS}
)

add_executable(pcl_to_octomap
  src/pcl2octomap.cpp
)

target_link_libraries(pcl_to_octomap
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(pcl2color_octomap
  src/pcl2color_octomap.cpp
)

target_link_libraries(pcl2color_octomap
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

install(TARGETS lslidar_3d_slam
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

