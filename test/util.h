#ifndef UTIL_H__
#define UTIL_H__

/* utility functions and useful typedefines */

#include <float.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

#define MAXDOUBLE DBL_MAX

// #define FULL_CONNECT

#ifdef FULL_CONNECT
#define NUMOFDIRS 26
#else
#define NUMOFDIRS 6
#endif

// #define sqrt2 1.414f
// #define sqrt3 1.732f
#define sqrt2 1.415f
#define sqrt3 1.733f
#define cost_inf DBL_MAX

namespace CF_PLAN {

constexpr size_t BOUND_SIZE = 6;

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

struct arrayHash {
  size_t operator()(const std::array<int, 3>& arr) const {
    size_t hx = std::hash<int>{}(arr[0]);
    size_t hy = std::hash<int>{}(arr[1]) << 1;
    size_t hz = std::hash<int>{}(arr[2]) << 2;
    return (hx ^ hy) ^ hz;
  }
};

}  // namespace CF_PLAN

#endif