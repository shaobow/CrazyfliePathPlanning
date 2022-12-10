#ifndef NODEDSTAR_H__
#define NODEDSTAR_H__

#include <math.h>

#include <array>
#include <iostream>
#include <utility>
#include <vector>

#include "util.h"

using namespace std;

namespace CF_PLAN {

#ifdef FULL_CONNECT
// 26-connected grid
int dX[NUMOFDIRS] = {0,  0,  1, 0, 1,  1,  1, -1, 0,  0, -1, -1, 0,
                     -1, -1, 1, 1, -1, -1, 1, -1, -1, 0, 0,  1,  1};
int dY[NUMOFDIRS] = {0,  1, 0,  1, 0,  1, 1,  0, -1, 0,  -1, 0,  -1,
                     -1, 1, -1, 1, -1, 1, -1, 0, 1,  -1, 1,  -1, 0};
int dZ[NUMOFDIRS] = {1,  0, 0, 1,  1, 0,  1,  0, 0, -1, 0,  -1, -1,
                     -1, 1, 1, -1, 1, -1, -1, 1, 0, 1,  -1, 0,  -1};
double cost[NUMOFDIRS] = {1,     1,     1,     sqrt2, sqrt2, sqrt2, sqrt3,
                          1,     1,     1,     sqrt2, sqrt2, sqrt2, sqrt3,
                          sqrt3, sqrt3, sqrt3, sqrt3, sqrt3, sqrt3, sqrt2,
                          sqrt2, sqrt2, sqrt2, sqrt2, sqrt2};
#else
// 6-connected grid
int dX[NUMOFDIRS] = {0, 0, 1, 0, 0, -1};
int dY[NUMOFDIRS] = {0, 1, 0, 0, -1, 0};
int dZ[NUMOFDIRS] = {1, 0, 0, -1, 0, 0};
double cost[NUMOFDIRS] = {1, 1, 1, 1, 1, 1};
#endif

class nodeDstar {
 private:
  int x;
  int y;
  int z;

  double g_value = MAXDOUBLE;
  double rhs_value = MAXDOUBLE;
  pair<double, double> key;

 public:
  nodeDstar(int x, int y, int z);
  ~nodeDstar() = default;

  bool operator==(const nodeDstar& cell) const;

  void set_g_value(double g_value);
  void set_rhs_value(double rhs_value);
  void set_key(double k1, double k2);
  void set_key(pair<double, double> k);
  double calc_h_value(nodeDstar* n_start);

  int getX() const;
  int getY() const;
  int getZ() const;
  double get_g_value() const;
  double get_rhs_value() const;
  pair<double, double> get_key() const;

  void print_node() const;
};

}  // namespace CF_PLAN
#endif