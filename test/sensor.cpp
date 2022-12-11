#include "sensor.h"

namespace CF_PLAN {

Sensor::Sensor(const std::string& file_path, double grid_size,
               double margin_size)
    : static_world(file_path, grid_size, margin_size) {}

std::vector<Coord> Sensor::update_collision_world(const Coord& robot_state) {
  // bool flag = false;
  std::vector<Coord> collision_list = {};
  for (auto i = robot_state.x - range; i <= robot_state.x + range; i++) {
    for (auto j = robot_state.y - range; j <= robot_state.y + range; j++) {
      for (auto k = robot_state.z - range; k <= robot_state.z + range; k++) {
        Coord temp(i, j, k);
        if (static_world.is_ocp(temp)) {
          if (partial_map.count(temp) == 0) {
            // flag = true;
            partial_map.insert(temp);
            collision_list.push_back(temp);
          }
        }
      }
    }
  }
  return collision_list;
}

bool Sensor::is_valid(const Coord& robot_state) {
  auto world_size = static_world.get_world_size();
  if (robot_state.x < 0 || robot_state.x > world_size.x) {
    return false;
  }
  if (robot_state.y < 0 || robot_state.y > world_size.y) {
    return false;
  }
  if (robot_state.z < 0 || robot_state.z > world_size.z) {
    return false;
  }

  /**/
  if (partial_map.count(robot_state) > 0) {
    return false;
  }

  // // only for testing static world
  // if (static_world.is_ocp(robot_state)) {
  //   return false;
  // }

  /**/
  return true;
}

Coord Sensor::convert_point(const double& x, const double& y, const double& z) {
  return static_world.convert_point(x, y, z);
}

int Sensor::getRange() const { return this->range; }

std::vector<double> Sensor::convert_idx(const Coord& idx) {
  return static_world.convert_idx(idx);
}

}  // namespace CF_PLAN