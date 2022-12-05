#include <mex.h>

#include <iostream>

#include "chrono"
#include "matrix.h"
#include "planner.h"
#include "sensor.h"
#include "util.h"
#include "world.h"

/* Input Arguments */
#define IN prhs[0]

/* Output Arguments */
#define PATH plhs[0]

namespace {
const std::string MAP1_PATH = "./maps/map1.txt";
const std::string MAP2_PATH = "./maps/map2.txt";
const std::string MAP3_PATH = "./maps/map3.txt";
}  // namespace

vector<vector<double>> plan(int map_id) {
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
      map_path = MAP2_PATH;
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
  CF_PLAN::Planner astar(robot_x, robot_y, robot_z, goal_x, goal_y, goal_z,
                         map_path);
  auto stop = std::chrono::high_resolution_clock::now();
  auto construct_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Planner class takes " << construct_time.count() / 1000.0
            << " seconds to construct.\n";

  start = chrono::high_resolution_clock::now();
  astar.plan();
  stop = chrono::high_resolution_clock::now();
  auto solve_time = chrono::duration_cast<chrono::milliseconds>(stop - start);
  astar.printPath();
  auto solution = astar.getPath();
  std::cout << "Planner takes " << solve_time.count() / 1000.0
            << " seconds to find solution.\n";

  // std::string MAP_PATH = "./maps/map1.txt";
  // auto start = std::chrono::high_resolution_clock::now();
  // CF_PLAN::Sensor sensor;
  // auto stop = std::chrono::high_resolution_clock::now();
  // auto sensor_construct_time =
  //     std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  // std::cout << "Sensor class takes " << sensor_construct_time.count() /
  // 1000.0
  //           << " seconds to construct.\n";
  // auto test_point_1 = sensor.concert_point(0, 0, 0);
  // if (sensor.is_valid(test_point_1)) {
  //   std::cout << "pt1 true\n";
  // } else {
  //   std::cout << "pt1 false\n";
  // }
  // auto test_point_2 = sensor.concert_point(2, 2, 0);
  // if (sensor.is_valid(test_point_2)) {
  //   std::cout << "pt2 true\n";
  // } else {
  //   std::cout << "pt2 false\n";
  // }
  return solution;
}

/* MEX entry function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  /* argument check */
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("MATLAB:cudaAdd:inputmismatch",
                      "Input arguments must be 1!");
  }
  if (nlhs != 1) {
    mexErrMsgIdAndTxt("MATLAB:cudaAdd:outputmismatch",
                      "Output arguments must be 1!");
  }

  auto solution = plan(1);
  mwSize m = solution.size();
  mwSize n = 3;
  PATH = mxCreateDoubleMatrix(m, n, mxREAL);
  double *path_ptr = mxGetPr(PATH);

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      path_ptr[i * n + j] = solution[i][j];
    }
  }

  return;
}
