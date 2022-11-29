#include "sensor.h"

namespace CF_PLAN {

namespace {
constexpr std::string MAP_PATH = "";
}  // namespace

Sensor::Sensor() {
  // load static entire world map
  World(MAP_PATH);

  // init local collision world
  std::unique_ptr<btCollisionConfiguration> collision_config =
      std::make_unique<btDefaultCollisionConfiguration>();
  std::unique_ptr<btCollisionDispatcher> collision_dispatcher =
      std::make_unique<btCollisionDispatcher>(collision_config);
  std::unique_ptr<btBroadphaseInterface> collision_broadphase =
      std::make_unique<btBroadphaseInterface>();
  btVector3 worldAabbMin();
}

Sensor::~Sensor() {}

}  // namespace CF_PLAN