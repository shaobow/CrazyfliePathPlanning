#include "world.h"

namespace CF_PLAN {

namespace {
constexpr int MAX_OBJ_NUM = 5000;
constexpr double cfDetectRange = 1.0;
}  // namespace

World::World(const std::string& file_path) {
  // init local collision world
  std::unique_ptr<btCollisionConfiguration> collision_config =
      std::make_unique<btDefaultCollisionConfiguration>();
  std::unique_ptr<btCollisionDispatcher> collision_dispatcher =
      std::make_unique<btCollisionDispatcher>(collision_config.get());

  btVector3 worldAabbMin(world_bound[0], world_bound[1], world_bound[3]);
  btVector3 worldAabbMax(world_bound[4], world_bound[5], world_bound[6]);
  std::unique_ptr<btBroadphaseInterface> collision_broadphase =
      std::make_unique<bt32BitAxisSweep3>(worldAabbMin, worldAabbMax,
                                          MAX_OBJ_NUM,
                                          (btOverlappingPairCache*)0, true);
  static_world = std::make_unique<btCollisionWorld>(collision_dispatcher.get(),
                                                    collision_broadphase.get(),
                                                    collision_config.get());

  // init robot object
  btBoxShape* robot_shape =
      std::make_unique<btBoxShape>(
          btVector3(cfDetectRange, cfDetectRange, cfDetectRange))
          .get();
  static_robot->setCollisionShape(robot_shape);

  // Add the collision objects to our collision world
  static_world->addCollisionObject(static_robot.get());

  // load map
  load_world(file_path);
}

World::~World() {}

void World::load_world(const std::string& file_path) {}

Boundary World::get_bound() const { return world_bound; }

std::vector<std::unique_ptr<btCollisionObject>> World::get_newly_detected(
    const Coord& robot_state) {
  std::vector<std::unique_ptr<btCollisionObject>> obj_list;

  // update robot location
  btTransform btTrans;
  btTrans.setIdentity();
  btTrans.setOrigin(
      btVector3(robot_state.getX(), robot_state.getY(), robot_state.getZ()));
  static_robot->setWorldTransform(btTrans);

  // TODO: create partial obstacle from virtual range contact info
  // Perform collision detection
  static_world->performDiscreteCollisionDetection();

  int numManifolds = static_world->getDispatcher()->getNumManifolds();
  // For each contact manifold
  for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contactManifold =
        static_world->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject* obA =
        static_cast<const btCollisionObject*>(contactManifold->getBody0());
    const btCollisionObject* obB =
        static_cast<const btCollisionObject*>(contactManifold->getBody1());
    contactManifold->refreshContactPoints(obA->getWorldTransform(),
                                          obB->getWorldTransform());
    int numContacts = contactManifold->getNumContacts();
    // For each contact point in that manifold
    for (int j = 0; j < numContacts; j++) {
      // Get the contact information
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      btVector3 ptA = pt.getPositionWorldOnA();
      btVector3 ptB = pt.getPositionWorldOnB();
      double ptdist = pt.getDistance();
    }
  }

  return obj_list;
}
}  // namespace CF_PLAN
