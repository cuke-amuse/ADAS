#include <cmath>
#include <iostream>
#include <math.h>
#include <string>

// ros
#include "ros/ros.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"

// msg
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"

#include "carla_msgs/CarlaEgoVehicleControl.h"

#include "casper_auto_msgs/Waypoint.h"
#include "casper_auto_msgs/WaypointArray.h"

#include "trajectory_controller.h"

using namespace std;

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "trajectory_controller");
  ros::NodeHandle n("trajectory_controller");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); 

  ros::Rate rate(frequency_rate);

 int cnt=0;
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
