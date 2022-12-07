#ifndef SENSOR_H__
#define SENSOR_H__

#include <memory>
#include <vector>

#include "util.h"
#include "world.h"

namespace CF_PLAN {

class Sensor {
 private:
  // use partial flag
  bool use_partial;
  // World class
  World static_world;
  // Partially known map occupancy LUT
  std::unordered_set<Coord, coordHash> partial_map;

  // Sensor range in direction
  int range = 1;

 public:
  Sensor(const std::string& file_path, double grid_size, double margin_size,
         bool is_dstar);

  // update the collision world by adding newly detected part to the object list
  // as a new collision objects
  std::vector<Coord> update_collision_world(const Coord& robot_state);

  bool is_valid(const Coord& robot_state);

  Coord convert_point(const double& x, const double& y, const double& z);
  std::vector<double> convert_idx(const Coord& idx);
};

}  // namespace CF_PLAN

#endif
