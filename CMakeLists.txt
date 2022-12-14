cmake_minimum_required(VERSION 2.8.3)
project(curb)

find_package(catkin REQUIRED
  COMPONENTS
  grid_map_core
  pcl_conversions
  grid_map_ros
  message_filters
  tf
)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_aggregate src/aggregate.cpp src/helper.cpp)

target_link_libraries(${PROJECT_NAME}_aggregate
  yaml-cpp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
