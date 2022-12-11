#include <iostream>

#include "chrono"
// #include "planner.h"
#include "plannerDstar.h"
#include "sensor.h"
#include "util.h"
#include "world.h"

namespace {
const std::string MAP1_PATH = "./maps/map1.txt";
const std::string MAP2_PATH = "./maps/map2.txt";
const std::string MAP3_PATH = "./maps/map3.txt";
}  // namespace

vector<vector<double>> plan(int map_id, double grid_size, double margin_size) {
  std::string map_path;
  double robot_x, robot_y, robot_z;
  double goal_x, goal_y, goal_z;

  switch (map_id) {
    case 1:
      map_path = MAP1_PATH;
      robot_x = 0.0;
      robot_y = -4.9;
      robot_z = 0.2;
      goal_x = 6.0;
      goal_y = 17.0;
      goal_z = 5.0;
      break;

    case 3:
      map_path = MAP3_PATH;
      robot_x = 0.0;
      robot_y = 5.0;
      robot_z = 5.0;
      goal_x = 20.0;
      goal_y = 5.0;
      goal_z = 5.0;
      break;

    default:
      std::cout << "Map ID not exist\n";
      break;
  }

  auto start = std::chrono::high_resolution_clock::now();
  CF_PLAN::plannerDstar dstarLite(robot_x, robot_y, robot_z, goal_x, goal_y,
                                  goal_z, map_path, grid_size, margin_size);
  auto stop = std::chrono::high_resolution_clock::now();
  auto construct_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "D* Lite planner takes " << construct_time.count() / 1000.0
            << " seconds to construct.\n";

  start = chrono::high_resolution_clock::now();
  dstarLite.plan();
  stop = chrono::high_resolution_clock::now();
  auto solve_time = chrono::duration_cast<chrono::milliseconds>(stop - start);
  dstarLite.printPath();
  auto solution = dstarLite.getPath();
  std::cout << "D* Lite planner takes " << solve_time.count() / 1000.0
            << " seconds to find solution.\n";
  return solution;
}

int main() {
  int map_id = 3;
  double grid_size = 0.2;
  double margin_size = 0.2;

  auto solution = plan(map_id, grid_size, margin_size);
  return 0;
}