#include <mex.h>

#include <iostream>

#include "chrono"
#include "matrix.h"
#include "planner.h"
#include "plannerDstar.hpp"
#include "sensor.h"
#include "util.h"
#include "world.h"

/* Input Arguments */
#define MAP_ID prhs[0]
#define GRID_SIZE prhs[1]
#define MARGIN_SIZE prhs[2]

/* Output Arguments */
#define PATH plhs[0]

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
  CF_PLAN::Planner dstarLite(robot_x, robot_y, robot_z, goal_x, goal_y, goal_z,
                             map_path, grid_size, margin_size);
  auto stop = std::chrono::high_resolution_clock::now();
  auto construct_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Planner class takes " << construct_time.count() / 1000.0
            << " seconds to construct.\n";

  start = chrono::high_resolution_clock::now();
  dstarLite.plan();
  stop = chrono::high_resolution_clock::now();
  auto solve_time = chrono::duration_cast<chrono::milliseconds>(stop - start);
  dstarLite.printPath();
  auto solution = dstarLite.getPath();
  std::cout << "Planner takes " << solve_time.count() / 1000.0
            << " seconds to find solution.\n";
  return solution;
}

/* MEX entry function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  /* argument check */
  if (nrhs != 3) {
    mexErrMsgIdAndTxt("MATLAB:inputmismatch", "Input arguments must be 3!");
  }
  if (nlhs != 1) {
    mexErrMsgIdAndTxt("MATLAB:outputmismatch", "Output arguments must be 1!");
  }

  int map_id = (int)mxGetScalar(MAP_ID);
  double grid_size = mxGetScalar(GRID_SIZE);
  double margin_size = mxGetScalar(MARGIN_SIZE);

  auto solution = plan(map_id, grid_size, margin_size);
  mwSize m = solution.size();
  mwSize n = 3;
  PATH = mxCreateDoubleMatrix(m, n, mxREAL);
  double *path_ptr = mxGetPr(PATH);

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      path_ptr[j * m + i] = solution[i][j];
    }
  }

  return;
}
