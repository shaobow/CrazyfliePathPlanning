#ifndef SENSOR_H__
#define SENSOR_H__

#include <vector>

#include "Collision/CollisionSdkC_Api.h"
#include "util.h"

namespace CF_PLAN {

class Sensor {
 private:
  // local bullet collision world
  plCollisionWorldHandle collision_world;
  std::vector<plCollisionObjectHandle> collision_objects_list;

 public:
  Sensor(/* args */);
  ~Sensor();

  // update the collision world by adding newly detected part to the object list
  // as a new collision objects
  bool update_collision_world(Coord robot_state);
};

}  // namespace CF_PLAN

#endif
