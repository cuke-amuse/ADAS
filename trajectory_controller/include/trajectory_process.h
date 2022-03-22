#ifndef TRAJECTORY_PROCESS_H
#define TRAJECTORY_PROCESS_H

#include <vector>

#include "ros/ros.h"

#include "trajectory_controller.h"
#include "nav_msgs/Odometry.h"
#include "casper_auto_msgs/WaypointArray.h"

using namespace std;

class TrajectoryPro {
public:
   TrajectoryPro(ros::NodeHandle& nh)
   : nh_(nh) 
   {
       nh_ = nh;
       GetPara();
       InitSubscribeENV();

       current_pose.resize(3);
       current_pose.assign(3, 0.0);
       current_velocity.resize(3);
       prev_p.resize(2);
       prev_p.assign(2, 0.0);
       cmd.resize(3);
       
   };
   ~TrajectoryPro() {};
    void Process();
private:
    void InitSubscribeENV();
    void FinalWaypointsCallback(const casper_auto_msgs::WaypointArray::ConstPtr &msg);
    void CurrentOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void FilterWP(); 
    void NormalWP();
    void LatencyControl();
    void GetPara();
    void ReInit();
    ros::NodeHandle nh_;
    int cnt_{0};
    std::vector<double> current_pose;     // x, y, yaw
    std::vector<double> current_velocity; //(3); // vx, vy, magnitude
    std::vector<double> prev_p;  //(2);
    std::vector<double> cmd;  // (3);
    vector<vector<double>> final_waypoints;
    vector<double> wp_distance;
    vector<vector<double>> wp_interp;
    //double current_z = 0;
    double frequency_rate{20.0};
    std::unique_ptr<TrajectoryController> mc{nullptr};
    std::string role_name, control_method;
    double lookahead_t_mpc, lookahead_dist_mpc;
    double lag_throttle, lag_brake, lag_steering;
     bool add_latency{false};
    ros::Publisher vehicle_control_cmd_pub;
    ros::Subscriber current_odom_sub;
    ros::Subscriber final_waypoints_sub;
};
#endif