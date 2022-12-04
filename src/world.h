#ifndef WORLD_H__
#define WORLD_H__

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "util.h"

namespace CF_PLAN {

class World {
 private:
  // boundary of the satic world
  Boundary world_bound;
  // blocks info
  std::vector<std::vector<double>> block_info;
  // grid occupancy LUT
  std::unordered_set<Coord, coordHash> ocp_LUT;

  // load world map from predefined text file
  void load_world(const std::string& file_path);

  void load_test_world();

 public:
  World(const std::string& file_path);
  ~World();

  // get boundary
  Boundary get_bound() const;
};

}  // namespace CF_PLAN

#endif