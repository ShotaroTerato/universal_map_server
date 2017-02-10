cmake_minimum_required(VERSION 2.8.3)
project(universal_map_server)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
#  gird_map_cv
  grid_map_loader
  grid_map_msgs
  grid_map_ros
  grid_map_core
  grid_map_rviz_plugin
  grid_map_visualization
  roscpp
  sensor_msgs
  cmake_modules
  filters
  nav_msgs
)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
)

add_executable(universal_map_server 
src/imageToGridmap.cpp
src/map_convertor.cpp
src/main.cpp)

add_executable(image_publisher
src/image_publisher.cpp)

#add_executable(map_convertor
#src/map_convertor.cpp)

target_link_libraries(universal_map_server
   ${catkin_LIBRARIES}
)

target_link_libraries(image_publisher
   ${catkin_LIBRARIES}
)

#target_link_libraries(map_convertor
#  ${catkin_LIBRARIES}
#)