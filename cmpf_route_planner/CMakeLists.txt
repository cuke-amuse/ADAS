cmake_minimum_required(VERSION 3.0.2)

project(cmpf_route_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-g)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  cmpf_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  #lanelet2_map_server
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  nodelet
)

#related to locate node of type
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_node
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ../lanelet2_map_server/include
)

# Controller nodelet
add_executable(cmpf_route_planner_node
  src/route_planner_node.cpp
  src/route_planner_core.cpp
)
add_dependencies(cmpf_route_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cmpf_route_planner_node
  ${catkin_LIBRARIES}
)

