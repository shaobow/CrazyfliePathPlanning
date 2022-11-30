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

struct PointedObjHash {
  size_t operator()(node* const& n) const {
    size_t hx = std::hash<int>{}(n->getX());
    size_t hy = std::hash<int>{}(n->getY()) << 1;
    size_t hz = std::hash<int>{}(n->getZ()) << 2;
    return (hx ^ hy) ^ hz;
  }
};

struct PointedObjEq {
  bool operator()(node* const& lhs, node* const& rhs) const {
    return lhs == rhs && lhs->get_key() == rhs->get_key();
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
  unordered_set<node*, PointedObjHash, PointedObjEq> closedList;
  node* s_goal;
  node* s_start;

  // D* Lite
  int k_m;

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

  ~Planner();

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

  // TODO: implemenet A* search
  void publishPath(node* s_current) {}

  void computePath() {
    node* s_current = openList.top();
    while (openList.size() != 0) {
      if (s_current == s_start) {
        publishPath(s_current);
      }

      openList.pop();
      closedList.insert(s_current);

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
    // publish solution;
  }

  /*
  // Key -> Operator Overloading Functions
  bool ifSmallerKeys(pair<double, double>& lhs, pair<double, double>& rhs) {
    if (lhs.first < rhs.first)
      return true;
    else if (lhs.first == rhs.first && lhs.second < rhs.second)
      return true;
    return false;
  }
  */

  /* For checking priority_quete auto-update after key values are changed */
  /*
  bool ifEqualKeys(pair<double, double>& lhs, pair<double, double>& rhs) {
    return lhs.first == rhs.first && lhs.second == rhs.second;
  }

  // Priority_queue Utility Functions
  iterator find(priority_queue<node*, vector<node*>, compareKey>& U, node* u) {
    auto first = U.first();
    auto last = U.end();
    while (first != last) {
      if (**first == *u) return first;
      ++first;
    }
    return last;
  }

  // TODO: implement D* Lite
  pair<double, double> calculateKey(node* s) {
    s->estimate_h_value(this->s_start);
    double min_g_rhs =
        min(s->get_g_value(), s->get_rhs_value());  // min(g(s), rhs(s))
    s->set_key(min_g_rhs + s->get_h_value() + this->k_m,
               min_g_rhs);  // [min(g(s), rhs(s)) + h(s_start, s) + k_m;
                            // min(g(s), rhs(s))]
    return s->get_key();
  }

  void initialize() {
    // U = zero;
    this->k_m = 0;
    // for all s within S, rhs(s) = g(s) = inf.
    this->s_goal->set_rhs_value(0);  // rhs(s_goal) = 0 but g(s_goal) = inf.
    this->calculateKey(this->s_goal);
    this->openList.push(this->s_goal);
  }

  void updateVertex(node* u) {
    if (u->get_g_value() != u->get_rhs_value()) {
      if (this->find(this->openList, u) !=
          this->openList.end()) {  // u within U

      } else {  // u NOT in U
      }
    }
    if (this->find(this->openList, u) != this->openList.end()) {  // u within U
    }
  }

  void computeShortestPath() {
    pair<double, double> key_s_start = this->calculateKey(this->s_start);

    while (this->ifSmallerKeys(this->openList.top()->get_key(), key_s_start) ||
           this->s_start->get_rhs_value() > this->s_start->get_g_value()) {
      node* u = this->openList.top();
      pair<double, double> k_old = u->get_key();
      pair<double, double> k_new = this->calculateKey(u);

      if (this->ifSmallerKeys(k_old, k_new)) {
        // U.Update(u, k_new)
        if (this->ifEqualKeys(this->openList.top()->get_key(), k_new)) {
          cout << "** [Notes] priority_queue auto-updates key_values. **"
               << endl;
        } else {
          cout << "** [Notes] priority_queue DOES NOT auto-updates key_values. "
                  "**"
               << endl;
          this->openList.pop();
          this->openList.push(u);
        }
      } else if (u->get_g_value() > u->get_rhs_value()) {
        u->set_g_value(u->get_rhs_value());
        this->openList.pop();  // U.Remove(u)

        // for all s wihtin Pred(u)
        for (int dir = 0; dir < NUMOFDIRS; dir++) {
          node* s =
              new node(u->getX() + this->dX[dir], u->getY() + this->dY[dir],
                       u->getZ() + this->dZ[dir]);
          if (!(s == s_goal)) {
            int rhs_value_new = std::min(s->get_rhs_value(),
                                         this->cost[dir] + u->get_g_value());
            s->set_rhs_value(rhs_value_new);
          }
          this->updateVertex(s);
        }
      }
    }
  }

  void main() {
    node* s_last = this->s_start;  // s_last = s_start
    this->initialize();
    this->computeShortestPath();
    while (!(this->s_start == this->s_goal)) {
      // TODO: implement rest of D* Lite
    }
  }

  */
};
}  // namespace CF_PLAN

#endif