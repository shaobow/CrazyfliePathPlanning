#include "sensor.h"

namespace CF_PLAN {

namespace {
const std::string MAP_PATH = " ";
constexpr int MAX_OBJ_NUM = 5000;
constexpr double robot_size = 1e-2;  // half of width
}  // namespace

Sensor::Sensor() : static_world(MAP_PATH) {
  // init local collision world
  std::unique_ptr<btCollisionConfiguration> collision_config =
      std::make_unique<btDefaultCollisionConfiguration>();
  std::unique_ptr<btCollisionDispatcher> collision_dispatcher =
      std::make_unique<btCollisionDispatcher>(collision_config.get());

  btVector3 worldAabbMin(static_world.get_bound()[0],
                         static_world.get_bound()[1],
                         static_world.get_bound()[3]);
  btVector3 worldAabbMax(static_world.get_bound()[4],
                         static_world.get_bound()[5],
                         static_world.get_bound()[6]);
  std::unique_ptr<btBroadphaseInterface> collision_broadphase =
      std::make_unique<bt32BitAxisSweep3>(worldAabbMin, worldAabbMax,
                                          MAX_OBJ_NUM,
                                          (btOverlappingPairCache*)0, true);
  collision_world = std::make_unique<btCollisionWorld>(
      collision_dispatcher.get(), collision_broadphase.get(),
      collision_config.get());

  // init robot object
  btBoxShape* robot_shape = std::make_unique<btBoxShape>(
                                btVector3(robot_size, robot_size, robot_size))
                                .get();
  robot_obj->setCollisionShape(robot_shape);

  // Add the collision objects to our collision world
  collision_world->addCollisionObject(robot_obj.get());

  // robot location at origin
  btTransform btTrans;
  btTrans.setIdentity();
  btTrans.setOrigin(btVector3(0., 0., 0.));
  robot_obj->setWorldTransform(btTrans);
}

Sensor::~Sensor() {}

bool Sensor::update_collision_world(const Coord& robot_state) {
  // dummy obstacles for debug tests

  // Create two collision objects
  btCollisionObject* sphere_A = new btCollisionObject();
  btCollisionObject* sphere_B = new btCollisionObject();
  // Move each to a specific location
  sphere_A->getWorldTransform().setOrigin(
      btVector3((btScalar)2, (btScalar)1.5, (btScalar)0));
  sphere_B->getWorldTransform().setOrigin(
      btVector3((btScalar)2, (btScalar)0, (btScalar)0));
  // Create a sphere with a radius of 1
  btSphereShape* sphere_shape = new btSphereShape(1);
  // Set the shape of each collision object
  sphere_A->setCollisionShape(sphere_shape);
  sphere_B->setCollisionShape(sphere_shape);
  // Add the collision objects to our collision world
  collision_world->addCollisionObject(sphere_A);
  collision_world->addCollisionObject(sphere_B);

  // Perform collision detection
  collision_world->performDiscreteCollisionDetection();

  int numManifolds = collision_world->getDispatcher()->getNumManifolds();
  // For each contact manifold
  for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contactManifold =
        collision_world->getDispatcher()->getManifoldByIndexInternal(i);
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
  return false;
}

bool Sensor::is_valid(const Coord& robot_state) {
  // update robot location
  btTransform btTrans;
  btTrans.setIdentity();
  btTrans.setOrigin(
      btVector3(robot_state.getX(), robot_state.getY(), robot_state.getZ()));
  robot_obj->setWorldTransform(btTrans);

  // Perform collision detection
  collision_world->performDiscreteCollisionDetection();

  int numManifolds = collision_world->getDispatcher()->getNumManifolds();
  // For each contact manifold
  for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contactManifold =
        collision_world->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject* obA =
        static_cast<const btCollisionObject*>(contactManifold->getBody0());
    const btCollisionObject* obB =
        static_cast<const btCollisionObject*>(contactManifold->getBody1());
    contactManifold->refreshContactPoints(obA->getWorldTransform(),
                                          obB->getWorldTransform());
    int numContacts = contactManifold->getNumContacts();
    if (numContacts > 0) {
      return false;
    }
  }

  return true;
}

}  // namespace CF_PLAN