#ifndef PLANNERDSTAR_H__
#define PLANNERDSTAR_H__

#include <math.h>

#include <utility>
#include <vector>

#include "openList.hpp"
#include "sensor.h"
#include "util.h"

using namespace std;
namespace CF_PLAN {

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
// 6-connected grid
int dX[NUMOFDIRS] = {0, 0, 1, 0, 0, -1};
int dY[NUMOFDIRS] = {0, 1, 0, 0, -1, 0};
int dZ[NUMOFDIRS] = {1, 0, 0, -1, 0, 0};
double cost[NUMOFDIRS] = {1, 1, 1, 1, 1, 1};
#endif

class plannerDstar {
 private:
  array<int, 3> coord_goal;
  nodeDstar* s_goal;

  array<int, 3> coord_start;
  nodeDstar* s_start;

  array<int, 3> coord_last;
  nodeDstar* s_last;

  openList U;
  double km = 0.0;

  Sensor sensor;

  vector<vector<double>> solution;

  pair<double, double> calculateKey(nodeDstar* node_u);
  void initialize();
  void updateVertex(const array<int, 3>& coord_u);
  void computeShortestPath();

  void move_s_start();
  void reassign_s_last(); /* s_last = s_start */
  double get_new_edge_cost(const array<int, 3>& coord_lhs,
                           const array<int, 3>& coord_rhs, const int dir);
  Coord convert_Coord(const array<int, 3>& coord);
  array<int, 3> generate_neighbor(const array<int, 3>& coord_u, int dir);
  void update_due_2_edge_cost(const array<int, 3>& coord_updated);

 public:
  plannerDstar(double robot_x, double robot_y, double robot_z, double goal_x,
               double goal_y, double goal_z, const std::string& file_path,
               double grid_size, double margin_size);
  ~plannerDstar() = default;

  void plan();
  void printPath() const;
  vector<vector<double>> getPath() const;
};
}  // namespace CF_PLAN

#endif
