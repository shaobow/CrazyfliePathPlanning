#include <iostream>

#include "planner.h"

int main(int, char**) {
  //   // init local collision world
  //   std::unique_ptr<btCollisionConfiguration> collision_config =
  //       std::make_unique<btDefaultCollisionConfiguration>();
  //   std::unique_ptr<btCollisionDispatcher> collision_dispatcher =
  //       std::make_unique<btCollisionDispatcher>(collision_config.get());

  //   btVector3 worldAabbMin(-50, -50, -50);
  //   btVector3 worldAabbMax(50, 50, 50);
  //   std::unique_ptr<btBroadphaseInterface> collision_broadphase =
  //       std::make_unique<bt32BitAxisSweep3>(worldAabbMin, worldAabbMax, 5000,
  //                                           (btOverlappingPairCache*)0,
  //                                           true);
  //   auto collision_world = std::make_unique<btCollisionWorld>(
  //       collision_dispatcher.get(), collision_broadphase.get(),
  //       collision_config.get());

  //   // Create two collision objects
  //   btCollisionObject* box_A = new btCollisionObject();
  //   btCollisionObject* box_B = new btCollisionObject();
  //   btCollisionObject* range = new btCollisionObject();
  //   // Move each to a specific location
  //   box_A->getWorldTransform().setOrigin(
  //       btVector3((btScalar)0, (btScalar)2, (btScalar)0));
  //   box_B->getWorldTransform().setOrigin(
  //       btVector3((btScalar)2, (btScalar)0, (btScalar)0));
  //   range->getWorldTransform().setOrigin(
  //       btVector3((btScalar)10, (btScalar)10, (btScalar)10));
  //   // Create a sphere with a radius of 1
  //   btBoxShape* box_shape_A = new btBoxShape(btVector3(1, 1, 1));
  //   btBoxShape* box_shape_B = new btBoxShape(btVector3(1, 1, 1));
  //   btBoxShape* box_shape_range = new btBoxShape(btVector3(10, 10, 10));
  //   // Set the shape of each collision object
  //   box_A->setCollisionShape(box_shape_A);
  //   box_B->setCollisionShape(box_shape_B);
  //   range->setCollisionShape(box_shape_range);
  //   // Add the collision objects to our collision world
  //   collision_world->addCollisionObject(box_A);
  //   collision_world->addCollisionObject(box_B);
  //   collision_world->addCollisionObject(range);

  //   // Perform collision detection
  //   collision_world->performDiscreteCollisionDetection();

  //   int numManifolds = collision_world->getDispatcher()->getNumManifolds();
  //   // For each contact manifold
  //   for (int i = 0; i < numManifolds; i++) {
  //     btPersistentManifold* contactManifold =
  //         collision_world->getDispatcher()->getManifoldByIndexInternal(i);
  //     const btCollisionObject* obA =
  //         static_cast<const btCollisionObject*>(contactManifold->getBody0());
  //     const btCollisionObject* obB =
  //         static_cast<const btCollisionObject*>(contactManifold->getBody1());
  //     contactManifold->refreshContactPoints(obA->getWorldTransform(),
  //                                           obB->getWorldTransform());
  //     int numContacts = contactManifold->getNumContacts();
  //     std::cout << "Manifold #" << i << "\n";
  //     // For each contact point in that manifold
  //     for (int j = 0; j < numContacts; j++) {
  //       // Get the contact information
  //       btManifoldPoint& pt = contactManifold->getContactPoint(j);
  //       btVector3 ptA = pt.getPositionWorldOnA();
  //       btVector3 ptB = pt.getPositionWorldOnB();
  //       double ptdist = pt.getDance();
  //       std::cout << "The #" << j << " contact\n";
  //       std::cout << "contact point A:\n";
  //       std::cout << "x=" << ptA.getX() << ",y=" << ptA.getY()
  //                 << ",z=" << ptA.getZ() << "\n";
  //       std::cout << "contact point B:\n";
  //       std::cout << "x=" << ptB.getX() << ",y=" << ptB.getY()
  //                 << ",z=" << ptB.getZ() << "\n";
  //       std::cout << "dist = " << ptdist << "\n";
  //     }
  //   }
  int robot_x = 0, robot_y = 0, robot_z = 0;
  int goal_x = 1, goal_y = 0, goal_z = 0;
  CF_PLAN::Planner astar(robot_x, robot_y, robot_z, goal_x, goal_y, goal_z);
  astar.plan();
}
