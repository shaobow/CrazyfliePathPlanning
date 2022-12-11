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

#include "openList.h"
#include "sensor.h"

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

  vector<vector<int>> solution;

  openList U;
  double km = 0.0;

  Sensor sensor;  // sensor for local map update

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
  PlannerDstar(double robot_x, double robot_y, double robot_z, double goal_x,
               double goal_y, double goal_z, const std::string& file_path,
               double grid_size, double margin_size)
      : sensor(file_path, grid_size, margin_size) {
    auto robot = sensor.convert_point(robot_x, robot_y, robot_z);
    auto goal = sensor.convert_point(goal_x, goal_y, goal_z);

    this->updateRobotPose(robot.x, robot.y, robot.z);

    this->coord_goal = {goal.x, goal.y, goal.z};
    s_goal = U.getNode(this->coord_goal);
    idx_goal = U.umap[coord_goal];

    solution.clear();
  };

  ~PlannerDstar() = default;

  void updateRobotPose(int robot_x, int robot_y, int robot_z) {
    this->coord_start = {robot_x, robot_y, robot_z};
    s_start = U.getNode(this->coord_start);
    idx_start = U.umap[coord_start];
  }

  // D* Lite
  pair<double, double> calculateKey(array<int, 3>& coord_u) {
    nodeDstar* node_u = U.getNode(coord_u);
    double min_g_rhs = std::min(node_u->get_g_value(), node_u->get_rhs_value());
    return make_pair(min_g_rhs + node_u->calc_h_value(s_start) + km, min_g_rhs);
  }

  void updateVertex(array<int, 3> coord_u) {
    nodeDstar* node_u = U.getNode(coord_u);

    if (coord_u != coord_goal) {
      if (!sensor.is_valid(Coord(coord_u[0], coord_u[1], coord_u[2]))) {
        node_u->set_rhs_value(cost_inf);
      } else {
        double rhs_min = cost_inf;
        double rhs_tmp;

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

    U.isInOpenList_and_remove(coord_u);

    if (node_u->get_g_value() != node_u->get_rhs_value()) {
      U.insert(coord_u, calculateKey(coord_u));
    }
  }

  void computeShortestPath() {
    array<int, 3> coord_u = U.top();
    pair<double, double> key_u = U.topKey(coord_u);
    nodeDstar* node_u = U.getNode(coord_u);

    pair<double, double> k_old;
    pair<double, double> k_u_new;

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

      coord_u = U.top();
      key_u = U.topKey(coord_u);
      node_u = U.getNode(coord_u);
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
    cout << "**** 1st computeShortestPath() ****" << endl;

    vector<Coord> Coord_updated;
    array<int, 3> coord_updated;
    array<int, 3> coord_next;
    while (coord_start != coord_goal) {
      /* if(g(s_start) == inf) then there is no known path */
      if (s_start->get_g_value() == DBL_MAX) {
        cout << "**** NO KNOWN PATH ****" << endl;
        return;
      }

      // check if any edge cost changes Coord_updated
      Coord_updated = sensor.update_collision_world(
          Coord(coord_start[0], coord_start[1], coord_start[2]));
      if (Coord_updated.size() != 0) {
        km += s_last->calc_h_value(s_start);
        update_s_last_2_s_start();

        for (auto itr : Coord_updated) {
          coord_updated = {itr.x, itr.y, itr.z};

          updateVertex(coord_updated);  // cell of obstacle
          for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int predX = coord_updated[0] + dX[dir];
            int predY = coord_updated[1] + dY[dir];
            int predZ = coord_updated[2] + dZ[dir];

            updateVertex({predX, predY, predZ});
          }
        }

        computeShortestPath();
        num_replan++;
        cout << "**** RE-PLANED " << num_replan << " ****";
        cout << "s_start: " << coord_start[0] << ", " << coord_start[1] << ", "
             << coord_start[2] << endl;
      }

      // move coord_start to coord_next
      if (sensor.is_valid(
              Coord(coord_start[0], coord_start[1], coord_start[2]))) {
        array<int, 3> coord_succ_min = coord_start;
        double min_cost_and_g = DBL_MAX;
        double min_tmp;

        int succX;
        int succY;
        int succZ;

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
          succX = coord_start[0] + dX[dir];
          succY = coord_start[1] + dY[dir];
          succZ = coord_start[2] + dZ[dir];

          if (sensor.is_valid(Coord(succX, succY, succZ))) {
            min_tmp =
                cost[dir] + U.getNode({succX, succY, succZ})->get_g_value();

            if (min_tmp < min_cost_and_g) {
              min_cost_and_g = min_tmp;
              coord_succ_min = {succX, succY, succZ};
            }
          }
        }

        if (coord_succ_min != coord_start)
          update_s_start(coord_succ_min);
        else {
          cout << "**** TRAPPED ****" << endl;
          return;
        }
      } else {
        cout << "**** WHY ROBOT is already TRAPPED ****" << endl;
        return;
      }
    }

    cout << "**** REACH GOAL POSE ****" << endl;
    vector<int> xyz{coord_start[0], coord_start[1], coord_start[2]};
    solution.push_back(xyz);
  }

  void update_s_last_2_s_start() {
    coord_last = coord_start;
    idx_last = U.umap[coord_start];
    s_last = U.getNode(coord_start);
  }

  void update_s_start(array<int, 3>& coord_new) {
    vector<int> xyz{coord_start[0], coord_start[1], coord_start[2]};
    solution.push_back(xyz);

    // if (U.getNode(coord_new)->get_g_value() >
    //     U.getNode(coord_start)->get_g_value())
    //   cout << "**** SOMETHING WRONG ****" << endl;

    // if (U.getNode(coord_new)->get_g_value() !=
    //     U.getNode(coord_new)->get_rhs_value()) {
    //   cout << "**** SOMETHING WRONG (inconsistent state) ****" << endl;
    //   U.printAroundNode(coord_new);
    // }

    // if (U.getNode(coord_new)->get_g_value() == DBL_MAX &&
    //     U.getNode(coord_new)->get_rhs_value() == DBL_MAX) {
    //   cout << "**** SOMETHING WRONG (state g and f both inf.) ****" << endl;
    //   U.printAroundNode(coord_new);
    // }

    // if (U.getNode(coord_new)->get_g_value() == DBL_MAX) {
    //   cout << "**** SOMETHING WRONG (state g inf.) ****" << endl;
    //   U.printAroundNode(coord_new);
    // }

    // if (U.getNode(coord_new)->get_rhs_value() == DBL_MAX) {
    //   cout << "**** SOMETHING WRONG (state g inf.) ****" << endl;
    //   U.printAroundNode(coord_new);
    // }

    coord_start = coord_new;
    idx_start = U.umap[coord_start];
    s_start = U.getNode(coord_start);
  }

  void printPath() {
    int i = 0;
    for (auto pt : solution) {
      cout << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }
  }

  vector<vector<int>> getPath() { return this->solution; }

  /* TEST FUNCTION */
  void print_key(pair<double, double> key) const {
    cout << " key: <" << key.first << ", " << key.second << ">";
  }

  int num_replan = 0;

  const int sensor_range = sensor.getRange();
};
}  // namespace CF_PLAN

#endif