#include "plannerDstar.hpp"

namespace CF_PLAN {
plannerDstar::plannerDstar(double robot_x, double robot_y, double robot_z,
                           double goal_x, double goal_y, double goal_z,
                           const std::string& file_path, double grid_size,
                           double margin_size)
    : sensor(file_path, grid_size, margin_size) {
  auto robot = sensor.convert_point(robot_x, robot_y, robot_z);
  auto goal = sensor.convert_point(goal_x, goal_y, goal_z);

  this->coord_goal = {goal.x, goal.y, goal.z};
  this->s_goal = U.getNode(this->coord_goal);
  this->idx_goal = U.umap[coord_goal];

  this->coord_start = {robot.x, robot.y, robot.z};
  this->s_start = U.getNode(this->coord_start);
  this->idx_start = U.umap[this->coord_start];
}

// pair<double, double> plannerDstar::calculateKey(array<int, 3>& coord_u) {
//   nodeDstar* node_u = U.getNode(coord_u);
//   double min_g_rhs = std::min(node_u->get_g_value(),
//   node_u->get_rhs_value()); return make_pair(min_g_rhs +
//   node_u->calc_h_value(s_start) + km, min_g_rhs);
// }

pair<double, double> plannerDstar::calculateKey(nodeDstar* node_u) {
  double min_g_rhs = std::min(node_u->get_g_value(), node_u->get_rhs_value());
  return make_pair(min_g_rhs + node_u->calc_h_value(s_start) + km, min_g_rhs);
}

void plannerDstar::initialize() {
  s_goal->set_rhs_value(0.0);
  pair<double, double> key_s_goal(s_goal->calc_h_value(s_start), 0.0);
  U.insert(coord_goal, key_s_goal);
}

void plannerDstar::updateVertex(array<int, 3>& coord_u) {
  nodeDstar* node_u = U.getNode(coord_u);

  if (node_u->get_g_value() != node_u->get_rhs_value())
    U.insert(coord_u, calculateKey(node_u));  // U.update() and U.insert()
  else
    U.remove(coord_u);
}

void plannerDstar::computeShortestPath() {
  array<int, 3> coord_u = U.top();
  pair<double, double> key_u = U.topKey(coord_u);
  nodeDstar* node_u = U.getNode(coord_u);

  pair<double, double> k_old;
  pair<double, double> k_new;

  while (U.isSmallerkey(key_u, calculateKey(node_u)) ||
         s_start->get_rhs_value() > s_start->get_g_value()) {
    k_old = key_u;
    k_new = calculateKey(node_u);
    if (U.isSmallerkey(k_old, k_new))
      U.insert(coord_u, k_new);
    else if (node_u->get_g_value() > node_u->get_rhs_value()) {
      node_u->set_g_value(node_u->get_rhs_value());
      U.remove(coord_u);

      for (int dir = 0; dir < NUMOFDIRS; dir++) {
      }
    }

    coord_u = U.top();
    key_u = U.topKey(coord_u);
    node_u = U.getNode(coord_u);
  }
}

void plannerDstar::plan() {}

void plannerDstar::update_s_start(array<int, 3>& coord_new) {}

void plannerDstar::update_s_last() {}

void plannerDstar::printPath() const {}

vector<vector<double>> plannerDstar::getPath() const {}
}  // namespace CF_PLAN