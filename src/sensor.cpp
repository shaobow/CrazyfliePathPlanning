#include "sensor.h"

namespace CF_PLAN {

namespace {
const std::string MAP_PATH = "./maps/map3.txt";
constexpr int MAX_OBJ_NUM = 5000;
constexpr double robot_size = 5e-2;  // half of width
}  // namespace

// Sensor::Sensor() : static_world(MAP_PATH) {
//   // init local collision world
//   // collision_config = std::make_unique<btDefaultCollisionConfiguration>();
//   // collision_dispatcher =
//   //     std::make_unique<btCollisionDispatcher>(collision_config.get());

//   // btVector3 worldAabbMin(-50, -50, -50);
//   // btVector3 worldAabbMax(50, 50, 50);
//   // collision_broadphase = std::make_unique<bt32BitAxisSweep3>(
//   //     worldAabbMin, worldAabbMax, 5000, (btOverlappingPairCache*)0, true);
//   // collision_world = std::make_unique<btCollisionWorld>(
//   //     collision_dispatcher.get(), collision_broadphase.get(),
//   //     collision_config.get());
//   collision_world = static_world.get_static_world();

//   // // ONLY FOR TEST
//   // // Create two collision objects
//   // btCollisionObject* box_A = new btCollisionObject();
//   // btCollisionObject* box_B = new btCollisionObject();
//   // // Move each to a specific location
//   // box_A->getWorldTransform().setOrigin(
//   //     btVector3((btScalar)0, (btScalar)2, (btScalar)0));
//   // box_B->getWorldTransform().setOrigin(
//   //     btVector3((btScalar)2, (btScalar)0, (btScalar)0));
//   // btBoxShape* box_shape_A = new btBoxShape(btVector3(1, 1, 1));
//   // btBoxShape* box_shape_B = new btBoxShape(btVector3(1, 1, 1));
//   // // Set the shape of each collision object
//   // box_A->setCollisionShape(box_shape_A);
//   // box_B->setCollisionShape(box_shape_B);
//   // // Add the collision objects to our collision world
//   // collision_world->addCollisionObject(box_A, 2, 1);
//   // collision_world->addCollisionObject(box_B, 2, 1);

//   // init robot object
//   robot_obj = std::make_unique<btCollisionObject>();

//   // robot location at origin
//   btTransform btTrans;
//   btTrans.setIdentity();
//   btTrans.setOrigin(btVector3((btScalar)0., (btScalar)0., (btScalar)0.));
//   robot_obj->setWorldTransform(btTrans);

//   // set robot shape
//   btBoxShape* robot_shape =
//       new btBoxShape(btVector3(robot_size, robot_size, robot_size));
//   robot_obj->setCollisionShape(robot_shape);

//   // Add the collision objects to our collision world
//   collision_world->addCollisionObject(robot_obj.get(), 1, 2);

//   // Perform collision detection
//   collision_world->performDiscreteCollisionDetection();
// }

Sensor::Sensor() : static_world(MAP_PATH) {}

bool Sensor::update_collision_world(const Coord& robot_state) { 
  bool flag = false;
  for(auto i=robot_state.x-range; i<robot_state.x+range; i++){
    for(auto j=robot_state.y-range; j<robot_state.y+range; j++){
      for(auto k=robot_state.z-range; k<robot_state.z+range; k++){
        Coord temp(i,j,k);
        if(static_world.is_ocp(temp)){
          if(partial_map.count(temp) == 0){
            flag = true;
            partial_map.insert(temp);
          }
        }
      }
    }
  }
  return flag;


  }

bool Sensor::is_valid(const Coord& robot_state) {
  if (partial_map.count(robot_state) > 0) {
    return false;
  }
  return true;
}

}  // namespace CF_PLAN