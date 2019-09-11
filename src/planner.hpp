#pragma once

#include <vector>
#include <array>
#include <limits>
#include <algorithm>
#include <iostream>
#include <iomanip>

#include <cmath>

#include "spline.h"


constexpr double kLaneWidth = 4.0;
constexpr unsigned int kLaneNum = 3;

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

  std::vector<Cartesian> ToCartesianVector() const {
    const std::size_t N = x.size();

    std::vector<Cartesian> vec(N);

    for (std::size_t i = 0; i < N; ++i) {
      vec.at(i) = {x.at(i), y.at(i)};
    }

    return vec;
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

template <typename T>
inline double norm(const T& p) {
  return sqrt(p.x * p.x + p.y * p.y);
}

template <typename T, typename U>
inline double distance(const T& p1, const U& p2) {
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return norm(Cartesian{dx, dy});
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

inline int GetLaneIndex(const double d) {
  if (d < 0.0 || d >= kLaneWidth * kLaneNum) { return -1; }
  return static_cast<int>(d / kLaneWidth);
}

inline double GetLaneMid(const int i) {
  return i * kLaneWidth + 0.5 * kLaneWidth;
}


class Planner {
  struct HaveCloseVehicles {
    bool on_ahead = false;
    bool on_left  = false;
    bool on_right = false;
  };

 public:
  static constexpr double kMaxSpeed = 22.2;
  static constexpr unsigned int kBaseMoveTimes = 50;
  static constexpr double kMaxAccel = 10.0;
  static constexpr double kMaxJerk = 10.0;

  static constexpr double kCloseDistance = 20.0;
  static constexpr std::size_t kPathPointNum = 50;

  Planner(const std::vector<Waypoint>& map_waypoints)
    : map_waypoints(map_waypoints)
  {}

  CartesianPath
  GenerateNextPathPoints(const CarState& car_state,
                         const std::vector<Cartesian>& prev_pts,
                         const Frenet& end_pt,
                         const std::vector<Vehicle> vehicles)
  {
    const std::size_t prev_size = prev_pts.size();

#ifdef DEBUG
    std::cout << "  yaw: " << car_state.yaw << std::endl;
#endif

    // Check other vehicles closeness
    HaveCloseVehicles close = CheckVehiclesCloseness(end_pt, vehicles, prev_size);

    // TODO: make decision on behavior
    intend_lane_index = GetLaneIndex(car_state.d);

    HeadedCartesian ref;
    std::vector<Cartesian> sparse_pts = GenerateSparsePoints(car_state, prev_pts, end_pt, ref);

    // Shift to car coordinates
    for (auto& p : sparse_pts) {
      const Cartesian shift{p.x - ref.x, p.y - ref.y};
      p = {shift.x * cos(-ref.theta) - shift.y * sin(-ref.theta),
           shift.x * sin(-ref.theta) + shift.y * cos(-ref.theta)};
    }
    const auto sparse_path = CartesianPath::fromCartesianVector(sparse_pts);

    tk::spline spline;
    spline.set_points(sparse_path.x, sparse_path.y);

    std::vector<Cartesian> next_pts(kPathPointNum);
    std::copy(prev_pts.begin(), prev_pts.end(), next_pts.begin());

    std::vector<Cartesian> extend_pts = GenerateInterporated(spline, ref, kPathPointNum - prev_size);
    std::copy(extend_pts.begin(), extend_pts.end(), next_pts.begin() + prev_size);

    return CartesianPath::fromCartesianVector(next_pts);
  }

 private:
  const std::vector<Waypoint> map_waypoints;
  int intend_lane_index = -1;
  double intend_speed = kMaxSpeed;

  HaveCloseVehicles
  CheckVehiclesCloseness(const Frenet& end_pt,
                         const std::vector<Vehicle>& vehicles,
                         const std::size_t prev_size) const
  {
    const int pred_car_lane_i = GetLaneIndex(end_pt.d);
    const int pred_car_s = end_pt.s;

    const double diff_sec = static_cast<double>(prev_size) / kBaseMoveTimes;

    HaveCloseVehicles close;
    for (const auto veh : vehicles) {
      const double pred_veh_s = veh.s + norm(Cartesian{veh.vx, veh.vy}) * diff_sec;
      const bool is_close = std::abs(pred_veh_s - pred_car_s) <= kCloseDistance;

      const int veh_lane_i = GetLaneIndex(veh.d);
      if (veh_lane_i < 0) { continue; }

      if (is_close) {
        if (veh_lane_i == pred_car_lane_i) {
          close.on_ahead = true;
        } else if (veh_lane_i == pred_car_lane_i - 1) {
          close.on_left = true;
        } else if (veh_lane_i == pred_car_lane_i + 1) {
          close.on_right = true;
        }
      }
    }

    return close;
  }

  std::vector<Cartesian>
  GenerateSparsePoints(const CarState car,
                       const std::vector<Cartesian>& prev_pts,
                       const Frenet& end_pt,
                       HeadedCartesian& ref)
  {
    const std::size_t prev_size = prev_pts.size();
    ref = {car.x, car.y, deg2rad(car.yaw)};

    std::vector<Cartesian> sparse_pts;
    if (prev_size < 2) {
      const Cartesian prev_car_pos{ref.x - cos(ref.theta),
                                   ref.y - sin(ref.theta)};
      sparse_pts.push_back(prev_car_pos);
      sparse_pts.push_back({ref.x, ref.y});
    } else {
      const auto cart_end_pt = prev_pts.at(prev_size - 1);
      ref.x = cart_end_pt.x;
      ref.y = cart_end_pt.y;

      const auto ref_prev = prev_pts.at(prev_size - 2);
      ref.theta = atan2(ref.y - ref_prev.y, ref.x - ref_prev.x);

      sparse_pts.push_back(ref_prev);
      sparse_pts.push_back({ref.x, ref.y});
    }

    const double s = prev_size > 0 ? end_pt.s : car.s;

    for (int i = 0; i < 3; ++i) {
      const Frenet fren_wp{s + (i+1) * 30.0, GetLaneMid(intend_lane_index)};
      const Cartesian cart_wp = ToCartesian(fren_wp, map_waypoints);
      sparse_pts.push_back(cart_wp);
    }

    return sparse_pts;
  }

  std::vector<Cartesian>
  GenerateInterporated(tk::spline& spline,
                       const HeadedCartesian& ref,
                       const std::size_t point_num)
  {
    std::vector<Cartesian> points(point_num);

    static constexpr double step = 30.0;
    const Cartesian target{step, spline(step)};
    const double target_dist = norm(target);
      const double N = target_dist / intend_speed * kBaseMoveTimes;

    for (std::size_t i = 0; i < point_num; ++i) {
      const double x = (i+1) * target.x / N;
      const Cartesian p_r = {x, spline(x)};
      const Cartesian p = {ref.x + p_r.x * cos(ref.theta) - p_r.y * sin(ref.theta),
                           ref.y + p_r.x * sin(ref.theta) + p_r.y * cos(ref.theta)};
      points.at(i) = p;
    }

    return points;
  }
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

