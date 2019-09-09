#pragma once

#include <vector>

class Planner {
 public:
  Planner(const std::vector<double>& map_waypoints_x,
          const std::vector<double>& map_waypoints_y,
          const std::vector<double>& map_waypoints_s,
          const std::vector<double>& map_waypoints_dx,
          const std::vector<double>& map_waypoints_dy)
    : map_waypoints_x(map_waypoints_x),
      map_waypoints_y(map_waypoints_y),
      map_waypoints_s(map_waypoints_s),
      map_waypoints_dx(map_waypoints_dx),
      map_waypoints_dy(map_waypoints_dy)
  {
  }

 private:
  const std::vector<double> map_waypoints_x;
  const std::vector<double> map_waypoints_y;
  const std::vector<double> map_waypoints_s;
  const std::vector<double> map_waypoints_dx;
  const std::vector<double> map_waypoints_dy;
};

