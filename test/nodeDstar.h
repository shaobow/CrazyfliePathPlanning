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

class nodeDstar {
 private:
  int x;
  int y;
  int z;

  double g_value = MAXDOUBLE;
  double rhs_value = MAXDOUBLE;
  pair<double, double> key;

 public:
  nodeDstar(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;

    this->key.first = MAXDOUBLE;
    this->key.second = MAXDOUBLE;
  };

  ~nodeDstar() = default;

  bool operator==(const nodeDstar& cell) const {
    return this->x == cell.x && this->y == cell.y && this->z == cell.z;
  }

  void set_g_value(double g_value) { this->g_value = g_value; }

  void set_rhs_value(double rhs_value) { this->rhs_value = rhs_value; }

  void set_key(double k1, double k2) {
    this->key.first = k1;
    this->key.second = k2;
  }

  void set_key(pair<double, double> k) { this->key = k; }

  double calc_h_value(nodeDstar* n_start) {
    return sqrt(pow(n_start->getX() - this->x, 2) +
                pow(n_start->getY() - this->y, 2) +
                pow(n_start->getZ() - this->z,
                    2));  // Euclidean distance in 3D
  }

  int getX() const { return this->x; }

  int getY() const { return this->y; }

  int getZ() const { return this->z; }

  double get_g_value() const { return this->g_value; }

  double get_rhs_value() const { return this->rhs_value; }

  pair<double, double> get_key() const { return this->key; }

  void print_node() const {
    cout << "node: " << this->x << ", " << this->y << ", " << this->z << endl;
  }
};

}  // namespace CF_PLAN
#endif