#ifndef PLANNERDSTAR_H__
#define PLANNERDSTAR_H__

#include <math.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

#include "nodeDstar.hpp"
#include "openList.hpp"
#include "sensor.h"
#include "util.h"

using namespace std;

namespace CF_PLAN {

class PlannerDstar {
 private:
  array<int, 3> coord_goal;
  array<int, 3> coord_start;

  nodeDstar* s_goal;
  nodeDstar* s_start;
  Idx idx_start;
  Idx idx_goal;

  array<int, 3> coord_last;
  nodeDstar* s_last;
  Idx idx_last;

  vector<nodeDstar*> solution;
  vector<vector<int>> solution_grid;

  openList U;
  double km = 0.0;

  // sensor for local map update
  Sensor sensor;

  bool isSmallerKey(const pair<double, double>& lhs,
                    const pair<double, double>& rhs) {
    if (lhs.first < rhs.first) return true;
    if (lhs.first == rhs.first && lhs.second < rhs.second) return true;
    return false;
  }

  friend bool operator!=(const array<int, 3>& s1, const array<int, 3>& s2) {
    if (s1[0] != s2[0]) return true;
    if (s1[1] != s2[1]) return true;
    if (s1[2] != s2[2]) return true;
    return false;
  }

 public:
  PlannerDstar(int robot_x, int robot_y, int robot_z, int goal_x, int goal_y,
               int goal_z) {
    this->updateRobotPose(robot_x, robot_y, robot_z);

    this->coord_goal = {goal_x, goal_y, goal_z};
    idx_goal = U.add_node(goal_x, goal_y, goal_z);
    s_goal = U.getNode(this->coord_goal);
  };

  ~PlannerDstar() = default;

  void updateRobotPose(int robot_x, int robot_y, int robot_z) {
    this->coord_start = {robot_x, robot_y, robot_z};
    idx_start = U.add_node(robot_x, robot_y, robot_z);
    s_start = U.getNode(this->coord_start);
  }

  // TODO: implement D* Lite
  pair<double, double> calculateKey(array<int, 3>& coord_u) {
    nodeDstar* node_u = U.getNode(coord_u);
    int min_g_rhs = std::min(node_u->get_g_value(), node_u->get_rhs_value());
    return make_pair(min_g_rhs + node_u->calc_h_value(s_start), min_g_rhs);
  }

  void updateVertex(array<int, 3> coord_u) {
    nodeDstar* node_u = U.getNode(coord_u);

    if (coord_u != coord_goal) {
      if (!is_valid(coord_u))
        node_u->set_rhs_value(cost_inf);
      else {
        int rhs_min = cost_inf;
        int rhs_tmp;

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
          int succX = coord_u[0] + dX[dir];
          int succY = coord_u[1] + dY[dir];
          int succZ = coord_u[2] + dZ[dir];

          if (sensor.is_valid(Coord(succX, succY, succZ))) {
            rhs_tmp =
                cost[dir] + U.getNode({succX, succY, succZ})->get_g_value();

            if (rhs_tmp < rhs_min) {
              rhs_min = rhs_tmp;
            }
          }
        }
        node_u->set_rhs_value(rhs_min);
      }
    }

    if (U.isInOpenList(coord_u)) U.remove(coord_u);

    if (node_u->get_g_value() != node_u->get_rhs_value())
      U.insert(coord_u, calculateKey(coord_u));
  }

  void computeShortestPath() {
    array<int, 3> coord_u = U.top();
    pair<double, double> key_u = U.topKey(coord_u);
    pair<double, double> k_old;

    pair<double, double> k_u_new;
    nodeDstar* node_u = U.getNode(coord_u);

    while (isSmallerKey(key_u, calculateKey(coord_start)) ||
           s_start->get_rhs_value() != s_start->get_g_value()) {
      k_old = key_u;
      U.pop(coord_u);

      k_u_new = calculateKey(coord_u);
      if (isSmallerKey(k_old, k_u_new)) {
        U.insert(coord_u, k_u_new);
      } else if (node_u->get_g_value() > node_u->get_rhs_value()) {
        node_u->set_g_value(node_u->get_rhs_value());

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
          int predX = coord_u[0] + dX[dir];
          int predY = coord_u[1] + dY[dir];
          int predZ = coord_u[2] + dZ[dir];

          updateVertex({predX, predY, predZ});
        }
      } else {
        node_u->set_g_value(DBL_MAX);

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
          int predX = coord_u[0] + dX[dir];
          int predY = coord_u[1] + dY[dir];
          int predZ = coord_u[2] + dZ[dir];

          updateVertex({predX, predY, predZ});
        }
        updateVertex(coord_u);
      }
    }
  }

  void initialize() {
    s_goal->set_rhs_value(0);
    U.insert(coord_goal, calculateKey(coord_goal));
  }

  void plan() {
    update_s_last_2_s_start();
    initialize();
    computeShortestPath();

    vector<Coord> Coord_updated;
    array<int, 3> coord_updated_temp;
    while (coord_start != coord_goal) {
      /* if(g(s_start) == inf) then there is no known path */
      if (s_start->get_g_value() == DBL_MAX) {
        cout << "**** NO KNOWN PATH ****" << endl;
        break;
      }

      // check if any edge cost changes
      Coord_updated = sensor.update_collision_world(Coord(coord_start));
      if (Coord_updated.size() != 0) {
        km += s_last->calc_h_value(s_start);
        update_s_last_2_s_start();

        for (auto itr : Coord_updated) {
          coord_updated_temp = U.node_list
        }
      }

      // move coord_start to coord_next
    }
  }

  void update_s_last_2_s_start() {
    coord_last = coord_start;
    idx_last = U.umap[coord_start];
    s_last = U.getNode(coord_start);
  }

  void mode_s_start() {}

  void backTrack(nodeDstar* s_current) {
    auto curr = s_current;
    solution.clear();

    solution_grid.clear();

    while (curr->get_back_ptr() != nullptr) {
      solution.push_back(curr);
      curr = curr->get_back_ptr();

      vector<int> xyz{curr->getX(), curr->getY(), curr->getZ()};
      solution_grid.push_back(xyz);
    }
    solution.push_back(curr);

    vector<int> xyz{curr->getX(), curr->getY(), curr->getZ()};
    solution_grid.push_back(xyz);
  }

  vector<vector<int>> getPath() { return this->solution_grid; }

  void printPath_grid() {
    int i = 0;
    for (auto xyz : solution_grid) {
      cout << "step " << i << ": ";
      cout << "x=" << xyz[0] << " "
           << "y=" << xyz[1] << " "
           << "z=" << xyz[2] << "\n";
      i++;
    }
  }
};
}  // namespace CF_PLAN

#endif