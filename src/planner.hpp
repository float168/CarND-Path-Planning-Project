#pragma once

#include <vector>
#include <array>
#include <iostream>


struct Cartesian {
  double x;
  double y;
};

struct CartesianPath {
  std::vector<double> x;
  std::vector<double> y;

  CartesianPath fromCartesianVector(const std::vector<Cartesian>& path) {
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


class Planner {
 public:
  Planner(const std::vector<Waypoint>& map_waypoints)
    : map_waypoints(map_waypoints)
  {}

  CartesianPath GenerateNextPathPoints(
    const CarState& car_state, const CartesianPath& previous_path,
    const Frenet& end_path, const std::vector<Vehicle> vehicles)
  {
    // TODO: writeup function
    //
    std::cout << vehicles.size() << " "
              << vehicles.at(0).id <<  " "
              << vehicles.at(0).x << " "
              << vehicles.at(0).y << std::endl;
    return CartesianPath{};
  }

 private:
  const std::vector<Waypoint> map_waypoints;
};


inline std::vector<Waypoint> combineIntoWaypoints(
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

inline std::vector<Vehicle> convertSensorFusion(
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

