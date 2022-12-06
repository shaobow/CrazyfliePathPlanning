#ifndef PLANNER_H__
#define PLANNER_H__

#include <math.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "node.h"
#include "sensor.h"
#include "util.h"

using namespace std;

namespace CF_PLAN {
#define FULL_CONNECT

#ifdef FULL_CONNECT
#define NUMOFDIRS 26
#else
#define NUMOFDIRS 6
#endif

#define sqrt2 1.414
#define sqrt3 1.732

class Planner {
 private:
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
  // 26-connected grid
  int dX[NUMOFDIRS] = {0, 0, 1, 0, 0, -1};
  int dY[NUMOFDIRS] = {0, 1, 0, 0, -1, 0};
  int dZ[NUMOFDIRS] = {1, 0, 0, -1, 0, 0};
  double cost[NUMOFDIRS] = {1, 1, 1, 1, 1, 1};
#endif

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

  vector<unique_ptr<node>> node_list;
  node* s_goal;
  node* s_start;
  Idx start_idx;
  Idx goal_idx;
  vector<vector<double>> solution;

  // sensor for local map update
  Sensor sensor;

  // check if successor state is valid
  bool isValid(const Coord& robot_state);

  int add_node(int x, int y, int z) {
    auto temp = make_unique<node>(x, y, z);
    node_list.push_back(move(temp));
    return node_list.size() - 1;
  }

 public:
  Planner(double robot_x, double robot_y, double robot_z, double goal_x,
          double goal_y, double goal_z, const std::string& file_path,
          double grid_size, double margin_size)
      : sensor(file_path, grid_size, margin_size) {
    auto robot = sensor.convert_point(robot_x, robot_y, robot_z);
    auto goal = sensor.convert_point(goal_x, goal_y, goal_z);
    this->updateRobotPose(robot.x, robot.y, robot.z);
    this->setGoalPose(goal.x, goal.y, goal.z);

    goal_idx = add_node(goal.x, goal.y, goal.z);
    s_goal = node_list[goal_idx].get();
    umap[{goal.x, goal.y, goal.z}] = goal_idx;

    start_idx = add_node(this->robotposeX, this->robotposeY, this->robotposeZ);
    s_start = node_list[start_idx].get();
    umap[{robotposeX, robotposeY, robotposeZ}] = start_idx;
  }

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

  int ifReachStart(node* currentPose) {
    return this->robotposeX == currentPose->getX() &&
           this->robotposeY == currentPose->getY() &&
           this->robotposeZ == currentPose->getZ();
  }

  void computePath() {
    auto compr = [this](const Idx& lhs, const Idx& rhs) {
      auto h1 = node_list[lhs]->get_h_value();
      auto h2 = node_list[rhs]->get_h_value();
      auto v1 = node_list[lhs]->get_f_value();
      auto v2 = node_list[rhs]->get_f_value();

      // smallest f value on the top
      if (v1 > v2) return true;
      // break tie with heuristic
      if (v1 == v2) {
        if (h1 > h2) return true;
      }
      return false;
    };

    priority_queue<Idx, vector<Idx>, decltype(compr)> openList(compr);

    openList.push(goal_idx);

    node* s_current = node_list[openList.top()].get();
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

        if (sensor.is_valid(Coord(newX, newY, newZ))) {
          // get unique node*
          Idx new_idx;
          if (umap.count({newX, newY, newZ}) == 0) {
            new_idx = add_node(newX, newY, newZ);
            umap[{newX, newY, newZ}] = new_idx;
          } else {
            new_idx = umap[{newX, newY, newZ}];
          }
          node* s_pred = node_list[new_idx].get();

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

  void plan() {
    s_goal->set_g_value(0);
    computePath();
  }

  void backTrack(node* s_current) {
    auto curr = s_current;
    solution.clear();

    while (curr->get_back_ptr() != nullptr) {
      curr = curr->get_back_ptr();

      Coord xyz_idx(curr->getX(), curr->getY(), curr->getZ());
      solution.push_back(sensor.convert_idx(xyz_idx));
    }
  }

  void printPath() {
    int i = 0;
    for (auto pt : solution) {
      cout << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
    }
  }

  vector<vector<double>> getPath() { return this->solution; }
};
}  // namespace CF_PLAN

#endif