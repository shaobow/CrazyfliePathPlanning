#ifndef PLANNERDSTAR_H__
#define PLANNERDSTAR_H__

#include <utility>

#include "openList.hpp"
#include "sensor.h"
#include "util.h"

using namespace std;
namespace CF_PLAN {
class plannerDstar {
 private:
  array<int, 3> coord_goal;
  nodeDstar* s_goal;
  Idx idx_goal;

  array<int, 3> coord_start;
  nodeDstar* s_start;
  Idx idx_start;

  array<int, 3> coord_last;
  nodeDstar* s_last;
  Idx idx_last;

  openList U;
  double km = 0.0;

  Sensor sensor;

  vector<vector<double>> solution;

 public:
  plannerDstar(double robot_x, double robot_y, double robot_z, double goal_x,
               double goal_y, double goal_z, const std::string& file_path,
               double grid_size, double margin_size);
  ~plannerDstar() = default;

  // pair<double, double> calculateKey(array<int, 3>& coord_u);
  pair<double, double> calculateKey(nodeDstar* node_u);
  void initialize();
  void updateVertex(array<int, 3>& coord_u);
  void computeShortestPath();
  void plan();

  void update_s_start(array<int, 3>& coord_new);
  void update_s_last();  // s_last = s_start

  void printPath() const;
  vector<vector<double>> getPath() const;
};
}  // namespace CF_PLAN

#endif
