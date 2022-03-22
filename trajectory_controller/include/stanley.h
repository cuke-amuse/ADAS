    vector<double> v1 = {waypoints[0][0] - x, waypoints[0][1] - y};
    vector<double> v2 = {cos(yaw), sin(yaw)};
    double heading_error = get_heading_error(waypoints, yaw);
    double cte_error =
        get_steering_direction(v1, v2) * get_cte_heading_error(v);
    steering = heading_error + cte_error;