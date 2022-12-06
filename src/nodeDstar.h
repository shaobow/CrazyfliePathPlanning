#ifndef NODEDSTAR_H__
#define NODEDSTAR_H__

#include <iostream>
#include <utility>
#include <vector>

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
  double g_value = DBL_MAX;
  double rhs_value = DBL_MAX;  // default: rhs(s) = g(s) = inf.
  pair<double, double> key;    // k(s) = [f(s); g*(s)]

  double cost;

  double h_value = -1;  // -1: h-value hasn't set

  // backtracking
  nodeDstar* backpointer = nullptr;  // backward from GOAL

 public:
  nodeDstar(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }

  ~nodeDstar() = default;

  int getX() const { return this->x; }

  int getY() const { return this->y; }

  int getZ() const { return this->z; }

  array<int, 3> getCoord() const { return {this->x, this->y, this->z}; }

  bool operator==(const nodeDstar& cell) const {
    return this->x == cell.getX() && this->y == cell.getY() &&
           this->z == cell.getZ();
  }

  void set_g_value(double g_value) { this->g_value = g_value; }

  void set_rhs_value(double rhs_value) { this->rhs_value = rhs_value; }

  void set_key(double k1, double k2) {
    this->key.first = k1;
    this->key.second = k2;
  }

  void set_key(pair<double, double> k) {
    this->key.first = k.first;
    this->key.second = k.second;
  }

  void set_back_ptr(nodeDstar* ptr) { this->backpointer = ptr; }

  double calc_h_value(nodeDstar* n_start) {
    return sqrt(pow(n_start->getX() - this->x, 2) +
                pow(n_start->getY() - this->y, 2) +
                pow(n_start->getZ() - this->z, 2));  // Euclidean distance in 3D
                                                     // this->h_value = 0;
  }

  double get_g_value() const { return this->g_value; }

  double get_rhs_value() const { return this->rhs_value; }

  pair<double, double> get_key() const { return this->key; }

  nodeDstar* get_back_ptr() const { return this->backpointer; }

  void print_node() const {
    cout << "node: " << this->x << ", " << this->y << ", " << this->z << endl;
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
};

}  // namespace CF_PLAN

#endif