#ifndef WORLD_H__
#define WORLD_H__

#include "util.h"

namespace CF_PLAN {

class World {
 private:
  // load world map from predefined text file
  bool load_world(const std::string& file_path);

 public:
  World(/* args */);
  ~World();
};

}  // namespace CF_PLAN

#endif