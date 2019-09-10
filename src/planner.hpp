#pragma once

#include <vector>
#include <array>
#include <iostream>
#include <limits>

#include <cmath>


struct Cartesian {
  double x;
  double y;
};

struct CartesianPath {
  std::vector<double> x;
  std::vector<double> y;

  static CartesianPath fromCartesianVector(const std::vector<Cartesian>& path) {
    const std::size_t N = path.size();

    CartesianPath self;

    self.x.resize(N);
    self.y.resize(N);

    for (std::size_t i = 0; i < N; ++i) {
      self.x.at(i) = path.at(i).x;
      self.y.at(i) = path.at(i).y;
    }

    return self;
  }
};

struct HeadedCartesian {
  double x;
  double y;
  double theta;
};

struct Frenet {
  double s;
  double d;
};

struct FrenetPath {
  std::vector<double> s;
  std::vector<double> d;
};

struct Waypoint {
  double x;
  double y;
  double s;
  double dx;
  double dy;
};

struct CarState {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
};

struct Vehicle {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};


constexpr double kPI = M_PI;
constexpr double k2PI = 2.0 * kPI;
constexpr double kPI_2 = 0.5 * kPI;

inline double deg2rad(double x) { return x * kPI / 180.0; }
inline double rad2deg(double x) { return x * 180.0 / kPI; }

template <typename T, typename U>
inline double distance(const T& p1, const U& p2) {
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return sqrt(dx * dx + dy * dy);
}


template <typename T>
inline std::size_t FindClosestWaypointIndex(
  const T& point,
  const std::vector<Waypoint>& map_waypoints)
{
  double closest_dist = std::numeric_limits<double>::max();
  std::size_t closest_index = 0;

  for (std::size_t i = 0; i < map_waypoints.size(); ++i) {
    const auto wp = map_waypoints.at(i);
    const double dist = distance(point, wp);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_index = i;
    }
  }

  return closest_index;
}

// Returns next waypoint of the closest waypoint
template <typename T>
inline std::size_t FindNextWaypointIndex(
  const T& headed_point,
  const std::vector<Waypoint>& map_waypoints)
{
  const std::size_t closest_index = GetClosestWaypointIndex(headed_point, map_waypoints);
  const Waypoint& closest_wp = map_waypoints.at(closest_index);

  const double heading = atan2((closest_wp.y - headed_point.y),
                               (closest_wp.x - headed_point.x));
  double angle = fabs(headed_point.theta - heading);
  angle = std::min(k2PI - angle, angle);

  std::size_t next_index = closest_index;

  if (angle > kPI_2) {
    ++next_index;
    if (next_index == map_waypoints.size()) {
      return 0;
    }
  }

  return next_index;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
template <typename T>
inline Frenet ToFrenet(const T& headed_point,
                       const std::vector<Waypoint>& map_waypoints)
{
  const std::size_t next_index = FindNextWaypointIndex(headed_point, map_waypoints);
  const Waypoint& next_wp = map_waypoints.at(next_index);

  const std::size_t prev_index = next_index == 0 ? map_waypoints.size() - 1 : next_index - 1;
  const Waypoint& prev_wp = map_waypoints.at(prev_index);

  const Cartesian norm{next_wp.x - prev_wp.x, next_wp.y - prev_wp.y};
  const Cartesian pos{headed_point.x - prev_wp.x, headed_point.y - prev_wp.y};

  // find the projection of x onto n
  const double proj_norm = (pos.x * norm.x + pos.y * norm.y)
                         / (norm.x * norm.x + norm.y * norm.y);
  const Cartesian proj{proj_norm * norm.x, proj_norm * norm.y};

  double frenet_d = distance(pos, proj);

  //see if d value is positive or negative by comparing it to a center point
  const Cartesian center{1000.0 - prev_wp.x, 2000.0 - prev_wp.y};
  const double center_to_pos  = distance(center, pos);
  const double center_to_proj = distance(center, proj);

  if (center_to_pos <= center_to_proj) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_index; ++i) {
    frenet_s += distance(map_waypoints.at(i), map_waypoints.at(i+1));
  }
  frenet_s += distance(Cartesian{0,0}, proj);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
template <typename T>
inline Cartesian ToCartesian(const T& point,
                             const std::vector<Waypoint>& map_waypoints)
{
  int prev_index = -1;
  while (point.s > map_waypoints.at(prev_index + 1).s &&
         prev_index < (int)(map_waypoints.size() - 1))
  {
    ++prev_index;
  }
  const Waypoint& prev_wp = map_waypoints.at(prev_index);

  std::size_t half_index = (prev_index + 1) % map_waypoints.size();
  const Waypoint& half_wp = map_waypoints.at(half_index);

  const double heading = atan2(half_wp.y - prev_wp.y, half_wp.x - prev_wp.x);
  // the x,y,s along the segment
  const double seg_s = point.s - prev_wp.s;

  const double seg_x = prev_wp.x + seg_s * cos(heading);
  const double seg_y = prev_wp.y + seg_s * sin(heading);

  const double perp_heading = heading - kPI_2;

  const double x = seg_x + point.d * cos(perp_heading);
  const double y = seg_y + point.d * sin(perp_heading);

  return {x, y};
}


class Planner {
 public:
  static constexpr double kMaxSpeed = 25.0;
  static constexpr unsigned int kBaseMoveTimes = 50;
  static constexpr double kMaxAccel = 10.0;
  static constexpr double kMaxJerk = 10.0;

  Planner(const std::vector<Waypoint>& map_waypoints)
    : map_waypoints(map_waypoints)
  {}

  CartesianPath GenerateNextPathPoints(
    const CarState& car_state, const CartesianPath& previous_path,
    const Frenet& end_path, const std::vector<Vehicle> vehicles)
  {
    std::vector<Cartesian> path;

    constexpr double base_step = kMaxSpeed / kBaseMoveTimes;
    for (int i = 0; i < kBaseMoveTimes; ++i) {
      const Frenet frenet{car_state.s + i * base_step, car_state.d};
      const Cartesian next = ToCartesian(frenet, map_waypoints);
      path.push_back(next);
    }

    return CartesianPath::fromCartesianVector(path);
  }

 private:
  const std::vector<Waypoint> map_waypoints;
};


inline std::vector<Waypoint> CombineIntoWaypoints(
  const std::vector<double>& map_waypoints_x,
  const std::vector<double>& map_waypoints_y,
  const std::vector<double>& map_waypoints_s,
  const std::vector<double>& map_waypoints_dx,
  const std::vector<double>& map_waypoints_dy)
{
  const std::size_t N = map_waypoints_x.size();

  if (map_waypoints_y.size()  != N ||
      map_waypoints_s.size()  != N ||
      map_waypoints_dx.size() != N ||
      map_waypoints_dy.size() != N)
  {
    throw "Mismatch in vector size";
  }

  std::vector<Waypoint> waypoints(N);

  for (std::size_t i = 0; i < N; ++i) {
    waypoints.at(i).x  = map_waypoints_x.at(i);
    waypoints.at(i).y  = map_waypoints_y.at(i);
    waypoints.at(i).s  = map_waypoints_s.at(i);
    waypoints.at(i).dx = map_waypoints_dx.at(i);
    waypoints.at(i).dy = map_waypoints_dy.at(i);
  }

  return waypoints;
}

inline std::vector<Vehicle> ConvertSensorFusion(
  const std::vector<std::array<double, 7>>& sensor_fusion)
{
  const std::size_t N = sensor_fusion.size();
  std::vector<Vehicle> vehicles(N);

  for (std::size_t i = 0; i < N; ++i) {
    vehicles.at(i).id = static_cast<decltype(Vehicle::id)>(sensor_fusion[i][0]);
    vehicles.at(i).x  = sensor_fusion[i][1];
    vehicles.at(i).y  = sensor_fusion[i][2];
    vehicles.at(i).vx = sensor_fusion[i][3];
    vehicles.at(i).vy = sensor_fusion[i][4];
    vehicles.at(i).s  = sensor_fusion[i][5];
    vehicles.at(i).d  = sensor_fusion[i][6];
  }

  return vehicles;
}

