cmake_minimum_required(VERSION 3.0.2)
project(lanelet2_map_server)

add_compile_options(-std=c++14 -O2 -Werror -Wall -Wno-unused -Wno-sign-compare)

set(CARLA_INCLUDE_DIRS /home/cuke/work/simulator-carla/carla-0911s/carla/LibCarla/source)
set(CARLA_RPC_INC_DIRS /home/cuke/work/simulator-carla/carla-0911s/carla/Examples/CppClient/libcarla-install/include/system)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  cmpf_msgs
  visualization_msgs
  tf2
  tf2_ros
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  nodelet
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_node
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

# Controller nodelet
add_executable(${PROJECT_NAME}_node
  src/lanelet2_map_server_node.cpp
  src/lanelet2_map_visualizer.cpp
  #src/route_planner_core.cpp
)

#related to msg def
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${CARLA_INCLUDE_DIRS}
  ${CARLA_RPC_INC_DIRS}
  ../router_planner/src/include
)

#############
## Install ##
#############

#install(TARGETS
 # ${PROJECT_NAME}_nodelet
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
 # DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(FILES 
#  lanelet2_map_server_nodelet_plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
