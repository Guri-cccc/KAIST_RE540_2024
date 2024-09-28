cmake_minimum_required(VERSION 3.0.2)
project(graph_planner)
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# add_library(path_finder src/dijkstra.cpp)
# add_dependencies(dijkstra ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(dijkstra
#     ${catkin_LIBRARIES}
# )

add_executable(${PROJECT_NAME}_node src/graph_planner.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
)