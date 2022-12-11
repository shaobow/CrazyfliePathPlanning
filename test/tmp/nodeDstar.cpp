#include "nodeDstar.hpp"

namespace CF_PLAN {
nodeDstar::nodeDstar(int x, int y, int z) {
  this->x = x;
  this->y = y;
  this->z = z;

  this->key.first = MAXDOUBLE;
  this->key.second = MAXDOUBLE;
}

bool nodeDstar::operator==(const nodeDstar& cell) const {
  return this->x == cell.x && this->y == cell.y && this->z == cell.z;
}

void nodeDstar::set_g_value(double g_value) { this->g_value = g_value; }

void nodeDstar::set_rhs_value(double rhs_value) { this->rhs_value = rhs_value; }

void nodeDstar::set_key(double k1, double k2) {
  this->key.first = k1;
  this->key.second = k2;
}

void nodeDstar::set_key(pair<double, double> k) { this->key = k; }

double nodeDstar::calc_h_value(nodeDstar* n_start) {
  return sqrt(pow(n_start->getX() - this->x, 2) +
              pow(n_start->getY() - this->y, 2) +
              pow(n_start->getZ() - this->z,
                  2));  // Euclidean distance in 3D
}

int nodeDstar::getX() const { return this->x; }

int nodeDstar::getY() const { return this->y; }

int nodeDstar::getZ() const { return this->z; }

double nodeDstar::get_g_value() const { return this->g_value; }

double nodeDstar::get_rhs_value() const { return this->rhs_value; }

pair<double, double> nodeDstar::get_key() const { return this->key; }

void nodeDstar::print_node() const {
  cout << "node: " << this->x << ", " << this->y << ", " << this->z << endl;
}

}  // namespace CF_PLAN