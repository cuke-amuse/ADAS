#include <ros/ros.h>
#include <memory>

#include "route_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "route_planner_node");
  ros::NodeHandle nh;
  // ros::console::levels::Debug
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  auto routePlanner = std::make_unique<cmpf::route_planner::RoutePlannerNodelet>(nh);
  routePlanner->onInit();
  routePlanner->Launch();
  return 0;
}