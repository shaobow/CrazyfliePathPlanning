#include "world.h"

#include <iostream>

namespace CF_PLAN {

namespace {
constexpr int MAX_OBJ_NUM = 5000;
constexpr double cfDetectRange = 1.0;
constexpr double MAP_MARGIN = 0.05;  // 5cm margin
}  // namespace

World::World(const std::string& file_path) {
  // load map
  load_world(file_path);

  // create obstacle map
  for (const auto& block : block_info) {
    std::vector<double> state_min = {block[0], block[1], block[3]};
    std::vector<double> state_max = {block[4], block[5], block[6]};
    auto xyz_start_idx = convert_point(
        block[0] - MAP_MARGIN, block[1] - MAP_MARGIN, block[2] - MAP_MARGIN);
    // auto x_coord_range = range2coord(state_min[0], state_max[0]);
    // auto y_coord_range = range2coord(state_min[1], state_max[1]);
    // auto z_coord_range = range2coord(state_min[2], state_max[2]);
    // for (int i = x_coord_range.first; i < x_coord_range.second; i++) {
    //   for (int j = y_coord_range.first; j < y_coord_range.second; j++) {
    //     for (int k = z_coord_range.first; k < z_coord_range.second; k++) {
    //       Coord curr_idx(i, j, k);
    //       ocp_LUT.insert(curr_idx);
    //     }
    //   }
    // }
  }
}

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

    // get grid world max sizes
    world_size.x = ceil(fabs(world_bound[3] - world_bound[0]) / GRID_SIZE);
    world_size.y = ceil(fabs(world_bound[4] - world_bound[1]) / GRID_SIZE);
    world_size.z = ceil(fabs(world_bound[5] - world_bound[2]) / GRID_SIZE);

    // get blocks info
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

Coord World::convert_point(const double& x, const double& y, const double& z) {
  int x_idx, y_idx, z_idx;
  x_idx = fmin(fmax(floor((x - world_bound[0]) / GRID_SIZE), 0), world_size.x);
  y_idx = fmin(fmax(floor((y - world_bound[1]) / GRID_SIZE), 0), world_size.y);
  z_idx = fmin(fmax(floor((z - world_bound[2]) / GRID_SIZE), 0), world_size.x);
  return Coord(x_idx, y_idx, z_idx);
}

bool World::is_ocp(Coord coord) const {
  if (ocp_LUT.count(coord) > 0) {
    return true;
  }
  return false;
}
}  // namespace CF_PLAN
