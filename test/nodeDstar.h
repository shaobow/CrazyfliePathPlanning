#ifndef NODEDSTAR_H__
#define NODEDSTAR_H__

// #include <bits/stdtr1c++.h>

#include <iostream>
#include <utility>
#include <vector>

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
  // location info
  int x;
  int y;
  int z;

  // D* Lite search
  double g_value = DBL_MAX;
  double rhs_value = DBL_MAX;  // default: rhs(s) = g(s) = inf.
  pair<double, double> key;    // k(s) = [f(s); g*(s)]

 public:
  nodeDstar(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;

    this->key.first = DBL_MAX;
    this->key.second = DBL_MAX;
  }

  ~nodeDstar() = default;

  int getX() const { return this->x; }

  int getY() const { return this->y; }

  int getZ() const { return this->z; }

  bool operator==(const nodeDstar& cell) const {
    return this->x == cell.getX() && this->y == cell.getY() &&
           this->z == cell.getZ();
  }

  void set_g_value(double g_value) { this->g_value = g_value; }

  void set_rhs_value(double rhs_value) { this->rhs_value = rhs_value; }

  void set_key(pair<double, double> k) {
    this->key.first = k.first;
    this->key.second = k.second;
  }

  double calc_h_value(nodeDstar* n_start) {
    return sqrt(pow(n_start->getX() - this->x, 2) +
                pow(n_start->getY() - this->y, 2) +
                pow(n_start->getZ() - this->z,
                    2));  // Euclidean distance in 3D
  }

  double get_g_value() const { return this->g_value; }

  double get_rhs_value() const { return this->rhs_value; }

  pair<double, double> get_key() const { return this->key; }

  void print_node() const {
    cout << "node: " << this->x << ", " << this->y << ", " << this->z << endl;
  }

  void print_key() const {
    cout << " key: " << this->key.first << ", " << this->key.second << endl;
  }

  bool isSmallerKey(const pair<double, double>& rhs) {
    if (this->key.first < rhs.first) return true;
    if (this->key.first == rhs.first && this->key.second < rhs.second)
      return true;
    return false;
  }

  bool isEqualKey(const pair<double, double>& rhs) {
    return this->key.first == rhs.first && this->key.second == rhs.second;
  }

  bool isGEqualRhs(nodeDstar* n) { return this->g_value != this->rhs_value; }
};

}  // namespace CF_PLAN

#endif