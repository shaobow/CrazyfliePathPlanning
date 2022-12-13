#include <iostream>

#include "chrono"
#include "planner.h"
#include "plannerDstar.h"
#include "sensor.h"
#include "util.h"
#include "world.h"

namespace {
const std::string MAP1_PATH = "./maps/map1.txt";
const std::string MAP2_PATH = "./maps/map2.txt";
const std::string MAP3_PATH = "./maps/map3.txt";
const std::string MAP4_PATH = "./maps/map_test.txt";
}  // namespace

vector<vector<int>> plan(int map_id, double grid_size, double margin_size) {
  std::string map_path;
  double robot_x, robot_y, robot_z;
  double goal_x, goal_y, goal_z;

  switch (map_id) {
    case 1:
      map_path = MAP1_PATH;
      robot_x = 0.0;
      robot_y = -5.0;
      robot_z = 0.2;
      goal_x = 6.0;
      goal_y = 18.0;
      goal_z = 2.0;
      break;

    case 2:
      map_path = MAP2_PATH;
      robot_x = 5.0;
      robot_y = 5.0;
      robot_z = 3.0;
      goal_x = 13.0;
      goal_y = 13.0;
      goal_z = 3.0;
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

    case 4:
      map_path = MAP4_PATH;
      robot_x = 8.0;
      robot_y = 0.2;
      robot_z = 2.0;
      goal_x = 15.0;
      goal_y = 18.0;
      goal_z = 7.0;
      break;

    default:
      std::cout << "Map ID not exist\n";
      break;
  }

  auto start = std::chrono::high_resolution_clock::now();
  CF_PLAN::PlannerDstar dstarLite(robot_x, robot_y, robot_z, goal_x, goal_y,
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
  std::cout << "map: " << map_id << ", connection: " << NUMOFDIRS
            << ", sensor range: " << dstarLite.sensor_range << endl;
  std::cout << "D* Lite planner takes " << solve_time.count() / 1000.0
            << " seconds to find solution.\n";
  std::cout << "Num of Replanning: " << dstarLite.num_replan << endl;
  std::cout << "Time for 1st plan: " << dstarLite.time_plan.count() / 1000.0
            << "; TIme for total replan: "
            << dstarLite.time_replan.count() / 1000.0 << endl;
  return solution;
}

void generateTxtFile(int map_id, vector<vector<int>>& solution) {
  int sensor_range = 1;
  int connection = 6;

  ofstream myfile;
  myfile.open("./results/tmp.txt");

  int i = 0;
  for (auto pt : solution) {
    myfile << pt[0] << "," << pt[1] << "," << pt[2] << "\n";
  }

  myfile.close();
}

int main() {
  int map_id = 4;
  double grid_size = 0.2;
  double margin_size = 0.2;

  auto solution = plan(map_id, grid_size, margin_size);

  generateTxtFile(map_id, solution);

  return 0;
}