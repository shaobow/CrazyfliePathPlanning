#ifndef UTIL_H__
#define UTIL_H__

/* utility functions and useful typedefines */

#include <btBulletCollisionCommon.h>

#include <array>
#include <string>
#include <vector>

namespace CF_PLAN {

constexpr size_t BOUND_CORNER_SIZE = 6;

const int CF_size = 0.10;
const int NUMOFDIRS = 26;

// 26-connected grid
const int dX[NUMOFDIRS] = {0,  0,  1, 0, 1,  1,  1, -1, 0,  0, -1, -1, 0,
                           -1, -1, 1, 1, -1, -1, 1, -1, -1, 0, 0,  1,  1};
const int dY[NUMOFDIRS] = {0,  1, 0,  1, 0,  1, 1,  0, -1, 0,  -1, 0,  -1,
                           -1, 1, -1, 1, -1, 1, -1, 0, 1,  -1, 1,  -1, 0};
const int dZ[NUMOFDIRS] = {1,  0, 0, 1,  1, 0,  1,  0, 0, -1, 0,  -1, -1,
                           -1, 1, 1, -1, 1, -1, -1, 1, 0, 1,  -1, 0,  -1};
const double cost[NUMOFDIRS] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};  // TODO: evaluate actual edge cost

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

using Idx = int;

}  // namespace CF_PLAN

#endif