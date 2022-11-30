#include <bits/stdtr1c++.h>
#include <float.h>

#include <iostream>
#include <utility>
#include <vector>

#define MAXDOUBLE DBL_MAX
#define w 1

using namespace std;

class node {
 private:
  // location info
  int x;
  int y;
  int z;

  // dynamic info
  double yaw;

  // time info
  // int time;

  // A* search info
  double g_value = MAXDOUBLE;
  double h_value = MAXDOUBLE;

  double f_value = g_value + h_value;

  // D* Lite search
  double rhs_value;
  pair<double, double> key;  // k(s) = [f(s); g*(s)]

  // backtracking
  node* backpointer = NULL;  // backward from GOAL

 public:
  node(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;

    this->rhs_value =
        this->g_value;  // default: for all s within S, rhs(s) = g(s) = inf.
  }

  int getX() const { return this->x; }

  int getY() const { return this->y; }

  int getZ() const { return this->z; }

  bool operator==(const node& rhs) const {
    return this->x == rhs.x && this->y == rhs.y && this->z == rhs.z;
  }

  void set_g_value(double previous_g_value, double edgeCost) {
    this->g_value =
        previous_g_value + edgeCost;  // TODO: g = g' + c or g = rhs' + c?????
  }

  // void set_g_value(double value_rhs) { this->g_value = value_rhs; }

  void set_g_value(double g_value) { this->g_value = g_value; }

  void estimate_h_value(node* n_start) {
    // TODO: times actual distance
    this->h_value = sqrt(
        pow(n_start->getX() - this->x, 2) + pow(n_start->getY() - this->y, 2) +
        pow(n_start->getZ() - this->z, 2));  // Euclidean distance in 3D
                                             // this->h_value = 0;
  }

  void set_f_value() { this->f_value = this->g_value + w * this->h_value; }

  void set_rhs_value(double rhs_value) { this->rhs_value = rhs_value; }

  void set_key(double k1, double k2) {
    this->key.first = k1;
    this->key.second = k2;
  }

  void set_back_ptr(node* ptr) { this->backpointer = ptr; }

  double get_g_value() const { return this->g_value; }

  double get_rhs_value() const { return this->rhs_value; }

  double get_h_value() const { return this->h_value; }

  double get_f_value() const { return this->f_value; }

  pair<double, double> get_key() const { return this->key; }
};