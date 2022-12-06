#include <iostream>

#include "plannerDstar.h"
//#include "planner.h"

int main(int, char**) {
  int robot_x = 0, robot_y = 0, robot_z = 1;
  int goal_x = 50, goal_y = 50, goal_z = 0;

  CF_PLAN::PlannerDstar dstar(robot_x, robot_y, robot_z, goal_x, goal_y,
                              goal_z);
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
