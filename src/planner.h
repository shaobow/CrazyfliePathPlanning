#ifndef PLANNER_H__
#define PLANNER_H__

#include <math.h>

#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <utility>

#include "node.hpp"
#include "sensor.h"

using namespace std;

namespace CF_PLAN {

#define NUMOFDIRS 26
#define crazyFlie_width 0.10
#define crazyFlie_length 0.10
#define crazyFlie_height 0.10
#define CF_size 0.10

struct compareKey {
  bool operator()(const node* lhs, const node* rhs) {
    pair<double, double> key1 = lhs->get_key();
    pair<double, double> key2 = rhs->get_key();

    if (key1.first > key2.first)
      return true;  // ordering in smallest key_1 value
    else if (key1.first == key2.first && key1.second > key2.second)
      return true;
    return false;
  }
};

struct compareFValue {
  bool operator()(const node* lhs, const node* rhs) {
    return lhs->get_f_value() > rhs->get_f_value();
  }
};

class Planner {
 private:
  // 26-connected grid
  int dX[NUMOFDIRS] = {0,  0,  1, 0, 1,  1,  1, -1, 0,  0, -1, -1, 0,
                       -1, -1, 1, 1, -1, -1, 1, -1, -1, 0, 0,  1,  1};
  int dY[NUMOFDIRS] = {0,  1, 0,  1, 0,  1, 1,  0, -1, 0,  -1, 0,  -1,
                       -1, 1, -1, 1, -1, 1, -1, 0, 1,  -1, 1,  -1, 0};
  int dZ[NUMOFDIRS] = {1,  0, 0, 1,  1, 0,  1,  0, 0, -1, 0,  -1, -1,
                       -1, 1, 1, -1, 1, -1, -1, 1, 0, 1,  -1, 0,  -1};
  double cost[NUMOFDIRS] = {
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};  // TODO: evaluate actual edge cost

  // goal info
  int goalposeX;
  int goalposeY;
  int goalposeZ;

  // current info
  int robotposeX;
  int robotposeY;
  int robotposeZ;

  // A* search
  priority_queue<node*, vector<node*>, compareFValue> openList;
  node* s_goal;
  node* s_start;
  vector<node*> solution;

  // sensor for local map update
  Sensor sensor;

  // check if successor state is valid
  bool isValid(const Coord& robot_state);

 public:
  Planner(int robot_x, int robot_y, int robot_z, int goal_x, int goal_y,
          int goal_z) {
    this->updateRobotPose(robot_x, robot_y, robot_z);
    this->setGoalPose(goal_x, goal_y, goal_z);
    this->s_goal = new node(this->goalposeX, this->goalposeY, this->goalposeZ);
    this->s_start =
        new node(this->robotposeX, this->robotposeY, this->robotposeZ);
  }

  ~Planner() = default;

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
    node* s_current = openList.top();
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

        if (sensor.is_valid(Coord((newX + 0.5) * CF_size,
                                  (newY + 0.5) * CF_size,
                                  (newZ + 0.5) * CF_size))) {
          node* s_pred = new node(newX, newY, newZ);
          if (s_pred->get_g_value() > (cost[dir] + s_current->get_g_value())) {
            s_pred->set_g_value(cost[dir] + s_current->get_g_value());
            s_pred->estimate_h_value(s_start);
            s_pred->set_f_value();
            s_pred->set_back_ptr(s_current);
            openList.push(s_pred);
          }
        }
      }

      s_current = openList.top();
    }
  }

  void plan() {
    s_goal->set_g_value(0);
    openList.push(s_goal);
    computePath();
  }

  void backTrack(node* s_current) {
    auto curr = s_current;
    solution.clear();
    while (curr->get_back_ptr() != nullptr) {
      solution.push_back(curr);
      curr = curr->get_back_ptr();
    }
    reverse(solution.begin(), solution.end());
  }

  void printPath() {
    int i = 0;
    for (auto node : solution) {
      cout << "step " << i;
      cout << "x=" << node->getX() << " "
           << "y=" << node->getY() << " "
           << "z=" << node->getZ() << "\n";
      i++;
    }
  }
};
}  // namespace CF_PLAN

#endif