#ifndef SENSOR_H__
#define SENSOR_H__

#include <memory>
#include <vector>

#include "util.h"
#include "world.h"

namespace CF_PLAN {

class Sensor {
 private:
  // static world
  World static_world;

  // local bullet collision world
  std::unique_ptr<btCollisionWorld> collision_world;
  std::vector<std::unique_ptr<btCollisionObject>> collision_objects_list;

 public:
  Sensor();
  ~Sensor();

  // update the collision world by adding newly detected part to the object list
  // as a new collision objects
  bool update_collision_world(const Coord& robot_state);
};

}  // namespace CF_PLAN

#endif
