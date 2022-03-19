#include <ros/ros.h>
#include "lanelet2_map_server.h"
#include <memory>

int main(int argc, char **argv) {
  ros::init(argc, argv, "lanelet_planning_node");
  ros::NodeHandle nh;
  // ros::console::levels::Debug
    std:: string path;
  path = argv[0];

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  auto mapServer = std::make_unique<cmpf::common::Lanelet2MapServerNodelet>(nh, path);

  mapServer->onInit(path);
  mapServer->Launch();
  return 0;
}