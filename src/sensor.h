#ifndef SENSOR_H__
#define SENSOR_H__

#include <memory>
#include <vector>

#include "util.h"
#include "world.h"

namespace CF_PLAN {

class Sensor {
 private:
  // World class
  World static_world;
  // Partially known map occupancy LUT
  std::unordered_set<Coord, coordHash> partial_map;
  
  // Sensor range in direction
  int range = 1;


 public:
  Sensor();
  ~Sensor();

  // update the collision world by adding newly detected part to the object list
  // as a new collision objects
  bool update_collision_world(const Coord& robot_state);

  bool is_valid(const Coord& robot_state);
};

}  // namespace CF_PLAN

#endif
