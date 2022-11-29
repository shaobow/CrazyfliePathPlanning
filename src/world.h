#ifndef WORLD_H__
#define WORLD_H__

#include "util.h"

namespace CF_PLAN {

class World {
 private:
  // static obstacles mesh
  Mesh obstacles;
  // boundary of the world
  Boundary world_bound;
  // load world map from predefined text file
  bool load_world(const std::string& file_path);

 public:
  World(const std::string& file_path);
  ~World();

  // get boundary
  Boundary get_bound() const;
};

}  // namespace CF_PLAN

#endif