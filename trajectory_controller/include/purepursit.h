
class PP : {
public :
    void proc(double kvf, double v, )
    {
            double lookahead_distance = kvf * v;
    int goal_idx = get_goal_waypoint_index(x, y, waypoints, lookahead_distance);
    vector<double> v1 = {waypoints[goal_idx][0] - x,
                         waypoints[goal_idx][1] - y};
    vector<double> v2 = {cos(yaw), sin(yaw)};
    double alpha = get_alpha(v1, v2, lookahead_distance);
    if (isnan(alpha))
      alpha = m_vars["alpha_previous"];
    if (!isnan(alpha))
      m_vars["alpha_previous"] = alpha;
    steering = get_steering_direction(v1, v2) *
               atan((2 * wheelbase * sin(alpha)) / (kpp * v));
    }
};