#ifndef NODEDSTAR_H__
#define NODEDSTAR_H__

#include <bits/stdtr1c++.h>
#include <float.h>

#include <iostream>
#include <utility>
#include <vector>

#define MAXDOUBLE DBL_MAX
constexpr double weight = 1.0;

using namespace std;

namespace CF_PLAN {

class nodeDstar {
 private:
  // location info
  int x;
  int y;
  int z;

  // dynamic info
  double yaw;

  // time info
  // int time;

  // D* Lite search
  double g_value = MAXDOUBLE;
  double h_value = MAXDOUBLE;
  double rhs_value;
  pair<double, double> key;  // k(s) = [f(s); g*(s)]

  // backtracking
  nodeDstar* backpointer = nullptr;  // backward from GOAL

 public:
  nodeDstar(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;

    this->rhs_value =
        this->g_value;  // default: for all s within S, rhs(s) = g(s) = inf.
  }

  ~nodeDstar() = default;

  int getX() const { return this->x; }

  int getY() const { return this->y; }

  int getZ() const { return this->z; }

  bool operator==(const nodeDstar& rhs) const {
    return this->x == rhs.getX() && this->y == rhs.getY() &&
           this->z == rhs.getZ();
  }

  void set_g_value(double g_value) { this->g_value = g_value; }

  double estimate_h_value(nodeDstar* n_start) {
    this->h_value = sqrt(
        pow(n_start->getX() - this->x, 2) + pow(n_start->getY() - this->y, 2) +
        pow(n_start->getZ() - this->z, 2));  // Euclidean distance in 3D
                                             // this->h_value = 0;

    return this->h_value;
  }

  void set_rhs_value(double rhs_value) { this->rhs_value = rhs_value; }

  void set_key(double k1, double k2) {
    this->key.first = k1;
    this->key.second = k2;
  }

  void set_back_ptr(nodeDstar* ptr) { this->backpointer = ptr; }

  double get_g_value() const { return this->g_value; }

  double get_h_value() const { return this->h_value; }

  double get_rhs_value() const { return this->rhs_value; }

  pair<double, double> get_key() const { return this->key; }

  nodeDstar* get_back_ptr() const { return this->backpointer; }

  void print_node() const {
    cout << "node: " << this->x << ", " << this->y << ", " << this->z << endl;
  }
};

}  // namespace CF_PLAN

#endif