#include <iostream>

//#include "plannerDstar.h"
#include "planner.h"

int main(int, char**) {
  int robot_x = 0, robot_y = -49, robot_z = 2;
  int goal_x = 60, goal_y = 170, goal_z = 5;

  CF_PLAN::Planner dstar(robot_x, robot_y, robot_z, goal_x, goal_y, goal_z);
  auto start = chrono::high_resolution_clock::now();
  dstar.plan();
  auto stop = chrono::high_resolution_clock::now();
  auto solve_time = chrono::duration_cast<chrono::milliseconds>(stop - start);
  // astar.printPath();
  dstar.printPath_grid();
  vector<vector<int>> solution = dstar.getPath();
  std::cout << "Planner takes " << solve_time.count() / 1000.0
            << " seconds to find solution.\n";
}
