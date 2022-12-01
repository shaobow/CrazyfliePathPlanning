#include "world.h"

#include <iostream>

namespace CF_PLAN {

namespace {
constexpr int MAX_OBJ_NUM = 5000;
constexpr double cfDetectRange = 1.0;
}  // namespace

World::World(const std::string& file_path) {
  // load map
  load_world(file_path);

  // init local collision world
  btVector3 worldAabbMin(world_bound[0], world_bound[1], world_bound[3]);
  btVector3 worldAabbMax(world_bound[4], world_bound[5], world_bound[6]);

  collision_config = std::make_unique<btDefaultCollisionConfiguration>();
  collision_dispatcher =
      std::make_unique<btCollisionDispatcher>(collision_config.get());
  collision_broadphase = std::make_unique<bt32BitAxisSweep3>(
      worldAabbMin, worldAabbMax, 5000, (btOverlappingPairCache*)0, true);
  static_world = std::make_unique<btCollisionWorld>(collision_dispatcher.get(),
                                                    collision_broadphase.get(),
                                                    collision_config.get());

  // add obstacles
  for (const auto& block : block_info) {
    // Create two collision objects
    btCollisionObject* box = new btCollisionObject();
    // Move each to a specific location
    box->getWorldTransform().setOrigin(
        btVector3((btScalar)block[0], (btScalar)block[1], (btScalar)block[2]));
    btBoxShape* box_shape =
        new btBoxShape(btVector3(block[3], block[4], block[5]));
    // Set the shape of each collision object
    box->setCollisionShape(box_shape);
    // Add the collision objects to our collision world
    static_world->addCollisionObject(box, 2, 1);
  }
}

World::~World() {}

void World::load_world(const std::string& file_path) {
  std::ifstream mapFile;
  mapFile.open(file_path.c_str());
  std::cout << file_path.c_str() << "\n";
  if (!mapFile.is_open()) {
    std::cout << "Can't open file" << std::endl;
  }
  while (mapFile) {
    std::string line;
    std::getline(mapFile, line);
    std::string var = line.substr(0, line.find_first_of(" "));

    if (var == "#" || line.length() == 0) {
      continue;
    }
    std::istringstream ss(line);
    std::string word;
    std::vector<std::string> word_list;
    while (ss >> word) {
      word_list.push_back(word);
    }
    double xmin, ymin, zmin, xmax, ymax, zmax;
    xmin = std::stod(word_list[1]);
    ymin = std::stod(word_list[2]);
    zmin = std::stod(word_list[3]);
    xmax = std::stod(word_list[4]);
    ymax = std::stod(word_list[5]);
    zmax = std::stod(word_list[6]);

    if (var == "boundary") {
      world_bound[0] = xmin;
      world_bound[1] = ymin;
      world_bound[2] = zmin;
      world_bound[3] = xmax;
      world_bound[4] = ymax;
      world_bound[5] = zmax;
    }
    std::vector<double> block_dim;
    if (var == "block") {
      double orx, yor, zor, dx, dy, dz;
      // origin
      orx = (xmax + xmin) / 2;
      yor = (ymax + ymin) / 2;
      zor = (zmax + zmin) / 2;
      dx = (xmax - xmin) / 2;
      dy = (ymax - ymin) / 2;
      dz = (zmax - zmin) / 2;
      // dimensions
      block_dim.push_back(orx);
      block_dim.push_back(yor);
      block_dim.push_back(zor);
      block_dim.push_back(dx);
      block_dim.push_back(dy);
      block_dim.push_back(dz);
      block_info.push_back(block_dim);
    }
  }
}

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

btCollisionWorld* World::get_static_world() const { return static_world.get(); }
}  // namespace CF_PLAN
