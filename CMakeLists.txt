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

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_generate src/generate_curb.cpp)
add_executable(${PROJECT_NAME}_downsample src/downsample.cpp)
add_executable(${PROJECT_NAME}_outlier src/outlier.cpp)
add_executable(${PROJECT_NAME}_make_ground src/make_ground.cpp)
add_executable(${PROJECT_NAME}_aggregate src/aggregate.cpp)

target_link_libraries(${PROJECT_NAME}_aggregate
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_generate
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_downsample
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_outlier
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_make_ground
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
