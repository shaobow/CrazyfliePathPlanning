#ifndef UTIL_H__
#define UTIL_H__

/* utility functions and useful typedefines */

#include <btBulletCollisionCommon.h>

#include <array>
#include <string>
#include <vector>

namespace CF_PLAN {

constexpr size_t BOUND_CORNER_SIZE = 6;

struct Vertex {
  double x;
  double y;
  double z;
};

using Boundary = std::array<double, BOUND_CORNER_SIZE>;

using Mesh = std::vector<Vertex>;

// structrue to store world info
struct Map {
  const std::vector<Mesh> obstacles;
  const Boundary bound;
};

// struct Coord {
//   double x;
//   double y;
//   double z;
// };
using Coord = btVector3;

}  // namespace CF_PLAN

#endif