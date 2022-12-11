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

  this->coord_start = {robot.x, robot.y, robot.z};
  this->s_start = U.getNode(this->coord_start);
}

pair<double, double> plannerDstar::calculateKey(nodeDstar* node_u) {
  double min_g_rhs = std::min(node_u->get_g_value(), node_u->get_rhs_value());
  return make_pair(min_g_rhs + node_u->calc_h_value(s_start) + km, min_g_rhs);
}

void plannerDstar::initialize() {
  s_goal->set_rhs_value(0.0);
  pair<double, double> key_s_goal(s_goal->calc_h_value(s_start), 0.0);
  U.insert(coord_goal, key_s_goal);
}

void plannerDstar::updateVertex(const array<int, 3>& coord_u) {
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

  while (U.isSmallerkey(key_u, calculateKey(s_start)) ||
         s_start->get_rhs_value() > s_start->get_g_value()) {
    k_old = key_u;
    k_new = calculateKey(node_u);
    if (U.isSmallerkey(k_old, k_new))
      U.insert(coord_u, k_new);
    else if (node_u->get_g_value() > node_u->get_rhs_value()) {
      node_u->set_g_value(node_u->get_rhs_value());
      U.remove(coord_u);

      nodeDstar* node_pred;
      array<int, 3> coord_pred;
      for (int dir = 0; dir < NUMOFDIRS; dir++) { /* s within Pred(u) */
        coord_pred = generate_neighbor(coord_u, dir);

        if (coord_pred != coord_goal) {
          node_pred = U.getNode(coord_pred);

          double cost_pred_2_u = get_new_edge_cost(coord_pred, coord_u, dir);

          node_pred->set_rhs_value(
              std::min(node_pred->get_rhs_value(),
                       cost_pred_2_u + node_u->get_g_value()));
        }
        updateVertex(coord_pred);
      }
    } else {
      int g_old = node_u->get_g_value();
      node_u->set_g_value(MAXDOUBLE);

      nodeDstar* node_s;
      array<int, 3> coord_s;
      array<int, 3> coord_s_prime;
      for (int dir_pred = 0; dir_pred < NUMOFDIRS + 1; dir_pred++) {
        /* s within Pred(u) U {u} */
        coord_s = generate_neighbor(coord_u, dir_pred);
        node_s = U.getNode(coord_s);

        if (node_s->get_rhs_value() ==
            get_new_edge_cost(coord_s, coord_u, dir_pred) + g_old) {
          if (coord_s != coord_goal) {
            double rhs_tmp = cost_inf;
            double rhs_min = rhs_tmp;

            for (int dir_succ = 0; dir_succ < NUMOFDIRS; dir_succ++) {
              /* s' within Succ(s) */
              coord_s_prime = generate_neighbor(coord_s, dir_succ);

              rhs_tmp = get_new_edge_cost(coord_s, coord_s_prime, dir_succ) +
                        U.getNode(coord_s_prime)->get_g_value();

              if (rhs_tmp < rhs_min) rhs_min = rhs_tmp;
            }
            node_s->set_rhs_value(rhs_min);
          }
        }
        updateVertex(coord_s);
      }
    }

    coord_u = U.top();
    key_u = U.topKey(coord_u);
    node_u = U.getNode(coord_u);
  }
}

void plannerDstar::plan() {
  reassign_s_last();
  initialize();
  computeShortestPath();

  while (coord_start != coord_goal) {
    /* if rhs(s_start) == inf., then there is no known path */
    if (s_start->get_rhs_value() == MAXDOUBLE) {
      cout << "**** NO KNOWN PATH ****" << endl;
      break;
    }

    move_s_start();

    vector<Coord> Coord_updated =
        sensor.update_collision_world(convert_Coord(coord_start));
    if (Coord_updated.size() != 0) {
      km += s_last->calc_h_value(s_start);
      reassign_s_last();

      /* for all directed edges (u, v) with changed edge costs */
      array<int, 3> coord_updated;  // coord_u
      array<int, 3> coord_nb;       // coord_u
      array<int, 3> coord_v;
      for (auto itr : Coord_updated) {
        /* STEP 1: update obstacle cell itself */
        coord_updated = {itr.x, itr.y, itr.z};
        for (int dir_v = 0; dir_v < NUMOFDIRS; dir_v++) {
          coord_v = generate_neighbor(coord_updated, dir_v);
          update_due_2_edge_cost(coord_updated, coord_v, dir_v,
                                 Coord_updated);  // 26 times
        }

        /* STEP 2: update neighbor */
        for (int dir_nb = 0; dir_nb < NUMOFDIRS; dir_nb++) {
          coord_nb = generate_neighbor(coord_updated, dir_nb);
          update_due_2_edge_cost(coord_nb, coord_updated, dir_nb,
                                 Coord_updated);  // 26 times
        }
      }
      computeShortestPath();
    }
  }
}

void plannerDstar::update_due_2_edge_cost(const array<int, 3>& coord_u,
                                          const array<int, 3>& coord_v,
                                          const int dir,
                                          const vector<Coord>& Coord_updated) {
  /* just update current state */
  nodeDstar* node_u = U.getNode(coord_u);
  nodeDstar* node_v = U.getNode(coord_v);

  double cost_old = get_old_edg_cost(coord_u, coord_v, dir, Coord_updated);
  double cost_new = get_new_edge_cost(coord_u, coord_v, dir);

  if (cost_old > cost_new) {
    if (coord_u != coord_goal) {
      node_u->set_rhs_value(
          std::min(node_u->get_rhs_value(), cost_new + node_v->get_g_value()));
    }
  } else if (node_u->get_rhs_value() == cost_old + node_v->get_g_value()) {
    if (coord_u != coord_goal) {
      array<int, 3> coord_s_prime;
      double rhs_tmp = cost_inf;
      double rhs_min = rhs_tmp;

      for (int dir_succ = 0; dir_succ < NUMOFDIRS; dir_succ++) {
        /* s' within Succ(s) */
        coord_s_prime = generate_neighbor(coord_u, dir_succ);

        rhs_tmp = get_new_edge_cost(coord_u, coord_s_prime, dir_succ) +
                  U.getNode(coord_s_prime)->get_g_value();

        if (rhs_tmp < rhs_min) rhs_min = rhs_tmp;
      }
      node_u->set_rhs_value(rhs_min);
    }
  }
  updateVertex(coord_u);
}

void plannerDstar::move_s_start() {
  array<int, 3> coord_tmp;
  array<int, 3> coord_succ = coord_start;  // in case can't find coord_tmp
  double cost_and_g_tmp = cost_inf;
  double cost_and_g_min = cost_and_g_tmp;
  double cost_to_s_prime;

  for (int dir_succ = 0; dir_succ < NUMOFDIRS; dir_succ++) {
    int succX = coord_start[0] + dX[dir_succ];
    int succY = coord_start[1] + dY[dir_succ];
    int succZ = coord_start[2] + dZ[dir_succ];

    coord_tmp = {succX, succY, succZ};
    cost_to_s_prime = get_new_edge_cost(coord_start, coord_tmp, dir_succ);
    cost_and_g_tmp = cost_to_s_prime + U.getNode(coord_tmp)->get_g_value();

    if (cost_and_g_tmp < cost_and_g_min) {
      cost_and_g_min = cost_and_g_tmp;
      coord_succ = coord_tmp;
    }
  }

  coord_start = coord_succ;
  s_start = U.getNode(coord_start);
}

void plannerDstar::reassign_s_last() { /* s_last = s_start; */
  coord_last = coord_start;
  s_last = s_start;
}

double plannerDstar::get_new_edge_cost(const array<int, 3>& coord_lhs,
                                       const array<int, 3>& coord_rhs,
                                       const int dir) {
  if (dir == NUMOFDIRS + 1) return 0.0;

  double cost_new = cost[dir];
  if (!sensor.is_valid(convert_Coord(coord_lhs)) ||
      !sensor.is_valid(convert_Coord(coord_rhs))) {
    cost_new = cost_inf;
  }
  return cost_new;
}

double plannerDstar::get_old_edg_cost(const array<int, 3>& coord_lhs,
                                      const array<int, 3>& coord_rhs,
                                      const int dir,
                                      const vector<Coord>& Coord_updated) {
  double cost_old = cost[dir];

  if (!sensor.is_valid(convert_Coord(coord_lhs)) &&
      !sensor.is_valid(convert_Coord(coord_rhs))) {
    if (std::find(Coord_updated.begin(), Coord_updated.end(),
                  convert_Coord(coord_lhs)) != Coord_updated.end() ||
        std::find(Coord_updated.begin(), Coord_updated.end(),
                  convert_Coord(coord_rhs)) != Coord_updated.end()) {
      cost_old = cost_inf;
    }
  }

  return cost_old;
}

Coord plannerDstar::convert_Coord(const array<int, 3>& coord) {
  return Coord(coord[0], coord[1], coord[2]);
}

array<int, 3> plannerDstar::generate_neighbor(const array<int, 3>& coord_u,
                                              const int dir) {
  if (dir == NUMOFDIRS + 1) return coord_u;
  return {coord_u[0] + dX[dir], coord_u[1] + dY[dir], coord_u[2] + dZ[dir]};
}

void plannerDstar::printPath() const {
  for (auto pt : solution) {
    cout << pt[0] << "," << pt[1] << "," << pt[2] << endl;
  }
}

vector<vector<double>> plannerDstar::getPath() const { return this->solution; }
}  // namespace CF_PLAN