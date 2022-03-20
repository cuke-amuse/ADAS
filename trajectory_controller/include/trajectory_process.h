#include <vector>

#include "trajectory_controller.h"
#include "nav_msgs/Odometry.h"

using namespace std;

#pragma once

class TrajectoryPro {
public:
   TrajectoryPro() {
       //mc  = std::make_unique<TrajectoryController>(control_method, lookahead_dist_mpc, lookahead_t_mpc);
       InitSubscribe();
   }
    void Process();
    void currentOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
private:
    void InitSubscribe();

    int cnt_{0};
    vector<double> current_pose(3);     // x, y, yaw
    vector<double> current_velocity(3); // vx, vy, magnitude
    vector<double> prev_p(2);
    //double current_z = 0;
    std::unique_ptr<TrajectoryController> mc{nullptr};
      string role_name, control_method;
  double lookahead_t_mpc, lookahead_dist_mpc;
};