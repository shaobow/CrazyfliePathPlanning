#ifndef WORLD_H__
#define WORLD_H__

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "util.h"

namespace CF_PLAN {

class World {
 private:
  // boundary of the satic world
  Boundary world_bound;
  // static collision world
  std::unique_ptr<btCollisionWorld> static_world;
  std::unique_ptr<btCollisionDispatcher> collision_dispatcher;
  std::unique_ptr<btCollisionConfiguration> collision_config;
  std::unique_ptr<btBroadphaseInterface> collision_broadphase;
  // robot object in static world
  std::unique_ptr<btCollisionObject> static_robot;

  std::vector<std::vector<double>> block_info;

  // load world map from predefined text file
  void load_world(const std::string& file_path);

  void load_test_world();

 public:
  World(const std::string& file_path);
  ~World();

  // get boundary
  Boundary get_bound() const;

  // check newly detected obstacles
  std::vector<std::unique_ptr<btCollisionObject>> get_newly_detected(
      const Coord& robot_state);

  btCollisionWorld* get_static_world() const;
};

}  // namespace CF_PLAN

#endif