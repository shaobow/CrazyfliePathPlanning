#include "sensor.h"

namespace CF_PLAN {

namespace {
const std::string MAP_PATH = " ";
constexpr int MAX_OBJ_NUM = 5000;
constexpr double robot_size = 1e-1;  // half of width
}  // namespace

Sensor::Sensor() {
  // init local collision world
  collision_config = std::make_unique<btDefaultCollisionConfiguration>();
  collision_dispatcher =
      std::make_unique<btCollisionDispatcher>(collision_config.get());

  btVector3 worldAabbMin(-50, -50, -50);
  btVector3 worldAabbMax(50, 50, 50);
  collision_broadphase = std::make_unique<bt32BitAxisSweep3>(
      worldAabbMin, worldAabbMax, 5000, (btOverlappingPairCache*)0, true);
  collision_world = std::make_unique<btCollisionWorld>(
      collision_dispatcher.get(), collision_broadphase.get(),
      collision_config.get());

  // ONLY FOR TEST
  // Create two collision objects
  btCollisionObject* box_A = new btCollisionObject();
  btCollisionObject* box_B = new btCollisionObject();
  // Move each to a specific location
  box_A->getWorldTransform().setOrigin(
      btVector3((btScalar)0, (btScalar)2, (btScalar)0));
  box_B->getWorldTransform().setOrigin(
      btVector3((btScalar)2, (btScalar)0, (btScalar)0));
  btBoxShape* box_shape_A = new btBoxShape(btVector3(1, 1, 1));
  btBoxShape* box_shape_B = new btBoxShape(btVector3(1, 1, 1));
  // Set the shape of each collision object
  box_A->setCollisionShape(box_shape_A);
  box_B->setCollisionShape(box_shape_B);
  // Add the collision objects to our collision world
  collision_world->addCollisionObject(box_A);
  collision_world->addCollisionObject(box_B);

  // init robot object
  robot_obj = std::make_unique<btCollisionObject>();

  // robot location at origin
  btTransform btTrans;
  btTrans.setIdentity();
  btTrans.setOrigin(btVector3((btScalar)0., (btScalar)0., (btScalar)0.));
  robot_obj->setWorldTransform(btTrans);

  // set robot shape
  btBoxShape* robot_shape =
      new btBoxShape(btVector3(robot_size, robot_size, robot_size));
  robot_obj->setCollisionShape(robot_shape);

  // Add the collision objects to our collision world
  collision_world->addCollisionObject(robot_obj.get());

  // Perform collision detection
  collision_world->performDiscreteCollisionDetection();
}

Sensor::~Sensor() {}

bool Sensor::update_collision_world(const Coord& robot_state) { return false; }

bool Sensor::is_valid(const Coord& robot_state) {
  // update robot location
  btTransform btTrans;
  btTrans.setIdentity();
  btTrans.setOrigin(btVector3((btScalar)robot_state.getX(),
                              (btScalar)robot_state.getY(),
                              (btScalar)robot_state.getZ()));
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