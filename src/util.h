#ifndef UTIL_H__
#define UTIL_H__

/* utility functions and useful typedefines */

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace CF_PLAN {

constexpr size_t BOUND_SIZE = 6;
constexpr double GRID_SIZE = 0.1;  // 10 cm grid size

struct Vertex {
  double x;
  double y;
  double z;
};

using Boundary = std::array<double, BOUND_SIZE>;

using Mesh = std::vector<Vertex>;

// structrue to store world info
struct Map {
  const std::vector<Mesh> obstacles;
  const Boundary bound;
};

struct Coord {
  int x;
  int y;
  int z;
  Coord() : x(0), y(0), z(0) {}
  Coord(int q1, int q2, int q3) : x(q1), y(q2), z(q3) {}
  bool operator==(const Coord& rhs) const {
    return (this->x == rhs.x && this->y == rhs.y && this->z == rhs.z);
  }
};

struct coordHash {
  size_t operator()(const Coord& coord) const {
    size_t hx = std::hash<int>{}(coord.x);
    size_t hy = std::hash<int>{}(coord.y) << 1;
    size_t hz = std::hash<int>{}(coord.z) << 2;
    return (hx ^ hy) ^ hz;
  }
};

using Idx = int;

// lhs is the smaller bound
auto range2coord = [](const double& lhs, const double& rhs) {
  int lhs_idx, rhs_idx;
  // left close right open interval
  if (lhs > 0) {
    lhs_idx = (int)((lhs - GRID_SIZE / 2) / GRID_SIZE);
  } else {
    lhs_idx = (int)((lhs - GRID_SIZE / 2) / GRID_SIZE);
  }
  if (rhs > 0) {
    rhs_idx = (int)((rhs + GRID_SIZE / 2) / GRID_SIZE);
  } else {
    rhs_idx = (int)((rhs + GRID_SIZE / 2) / GRID_SIZE);
  }

  return std::pair<int, int>(lhs_idx, rhs_idx);
};

}  // namespace CF_PLAN

#endif