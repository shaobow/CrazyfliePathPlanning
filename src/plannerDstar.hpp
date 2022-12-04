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

#include "nodeDstar.hpp"
#include "sensor.h"
#include "util.h"

using namespace std;

namespace CF_PLAN {

struct arrayHash {
  size_t operator()(const array<int, 3>& arr) const {
    size_t hx = std::hash<int>{}(arr[0]);
    size_t hy = std::hash<int>{}(arr[1]) << 1;
    size_t hz = std::hash<int>{}(arr[2]) << 2;
    return (hx ^ hy) ^ hz;
  }
};

class PlannerDstar {
 private:
  // goal info
  int goalposeX;
  int goalposeY;
  int goalposeZ;

  // current info
  int robotposeX;
  int robotposeY;
  int robotposeZ;

  // A* search
  unordered_map<array<int, 3>, int, arrayHash>
      umap;  // use coord. to find idx of ptr address

  vector<unique_ptr<nodeDstar>> node_list;
  nodeDstar* s_goal;
  nodeDstar* s_start;
  Idx idx_start;
  Idx idx_goal;
  vector<nodeDstar*> solution;
  vector<vector<int>> solution_grid;

  Idx idx_last;
  double km = 0.0;

  // sensor for local map update
  Sensor sensor;

  // check if successor state is valid
  bool isValid(const Coord& robot_state);
  bool isUpdate(const Coord& robot_state);

  int add_node(int x, int y, int z) {
    auto temp = make_unique<nodeDstar>(x, y, z);
    node_list.push_back(move(temp));
    return node_list.size() - 1;
  }

 public:
  PlannerDstar(int robot_x, int robot_y, int robot_z, int goal_x, int goal_y,
               int goal_z) {
    this->updateRobotPose(robot_x, robot_y, robot_z);
    this->setGoalPose(goal_x, goal_y, goal_z);

    idx_goal = add_node(goal_x, goal_y, goal_z);
    s_goal = node_list[idx_goal].get();
    umap[{goal_x, goal_y, goal_z}] = idx_goal;

    idx_start = add_node(this->robotposeX, this->robotposeY, this->robotposeZ);
    s_start = node_list[idx_start].get();
    umap[{robotposeX, robotposeY, robotposeZ}] = idx_start;
  }

  ~PlannerDstar() = default;

  void setGoalPose(int goal_x, int goal_y, int goal_z) {
    this->goalposeX = goal_x;
    this->goalposeY = goal_y;
    this->goalposeZ = goal_z;
  }

  void updateRobotPose(int robot_x, int robot_y, int robot_z) {
    this->robotposeX = robot_x;
    this->robotposeY = robot_y;
    this->robotposeZ = robot_z;
  }

  // int ifReachStart(nodeDstar* currentPose) {
  //   return this->robotposeX == currentPose->getX() &&
  //          this->robotposeY == currentPose->getY() &&
  //          this->robotposeZ == currentPose->getZ();
  // }

  // D* Lite Utility Functions
  bool ifSmallerKey(const pair<double, double>& lhs,
                    const pair<double, double>& rhs) {
    if (lhs.first < rhs.first) return true;
    if (lhs.first == rhs.first && lhs.second < rhs.second) return true;
    return false;
  }

  bool ifEqualKey(const pair<double, double>& lhs,
                  const pair<double, double>& rhs) {
    return lhs.first == rhs.first && lhs.second == rhs.second;
  }

  // TODO: implement D* Lite
  pair<double, double> calculateKey(const Idx& idx_s) {
    auto s = node_list[idx_s].get();
    int min_g_and_rhs = std::min(s->get_g_value(), s->get_rhs_value());
    s->set_key(min_g_and_rhs + s->estimate_h_value(s_start) + km,
               min_g_and_rhs);
    return s->get_key();
  }

  void computePath() {
    auto compr = [this](const Idx& lhs, const Idx& rhs) {
      auto k1 = node_list[lhs]->get_key();
      auto k2 = node_list[rhs]->get_key();

      if (k1.first > k2.first) return true;
      if (k1.first == k2.first && k1.second > k2.second) return true;
      return false;
    };

    // initialize()
    Idx idx_last = idx_start;
    priority_queue<Idx, vector<Idx>, decltype(compr)> openList(compr);
    s_goal->set_rhs_value(0);
    calculateKey(idx_goal);
    openList.push(idx_goal);

    // computeShortestPath()
    nodeDstar* s_current = node_list[openList.top()].get();
    pair<double, double> k_old = s_current->get_key();

    while (openList.size() != 0) {
      if (*s_current == *s_start) {
        cout << "** FOUND PATH **" << endl;
        backTrack(s_current);
        return;
      }

      openList.pop();

      for (int dir = 0; dir < NUMOFDIRS; dir++) {
        int newX = s_current->getX() + dX[dir];
        int newY = s_current->getY() + dY[dir];
        int newZ = s_current->getZ() + dZ[dir];

        if (sensor.is_valid(Coord((btScalar)(newX + 0.5) * CF_size,
                                  (btScalar)(newY + 0.5) * CF_size,
                                  (btScalar)(newZ + 0.5) * CF_size))) {
          // get unique node*
          Idx new_idx;
          if (umap.count({newX, newY, newZ}) == 0) {
            new_idx = add_node(newX, newY, newZ);
            umap[{newX, newY, newZ}] = new_idx;
          } else {
            new_idx = umap[{newX, newY, newZ}];
          }
          nodeDstar* s_pred = node_list[new_idx].get();

          if (s_pred->get_g_value() > (cost[dir] + s_current->get_g_value())) {
            s_pred->set_g_value(cost[dir] + s_current->get_g_value());
            s_pred->estimate_h_value(s_start);
            s_pred->set_f_value();
            s_pred->set_back_ptr(s_current);
            openList.push(new_idx);
          }
        }
      }

      s_current = node_list[openList.top()].get();
      // s_current->print_node();
    }
  }

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