#ifndef WORLD_H__
#define WORLD_H__

#include <memory>

#include "util.h"

namespace CF_PLAN {

class World {
 private:
  // boundary of the satic world
  Boundary world_bound;
  // static collision world
  std::unique_ptr<btCollisionWorld> static_world;
  // robot object in static world
  std::unique_ptr<btCollisionObject> static_robot;

  // load world map from predefined text file
  void load_world(const std::string& file_path);

 public:
  World(const std::string& file_path);
  ~World();

  // get boundary
  Boundary get_bound() const;

  // check newly detected obstacles
  std::vector<std::unique_ptr<btCollisionObject>> get_newly_detected(
      const Coord& robot_state);
};

}  // namespace CF_PLAN

#endif