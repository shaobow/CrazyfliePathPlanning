#include <mex.h>

#include <iostream>
#include <tuple>

#include "chrono"
#include "matrix.h"
#include "planner.h"
#include "plannerDstar.h"
#include "sensor.h"
#include "util.h"
#include "world.h"

/* Input Arguments */
#define MAP_ID prhs[0]
#define GRID_SIZE prhs[1]
#define MARGIN_SIZE prhs[2]
#define START prhs[3]
#define GOAL prhs[4]
#define USE_DSTAR prhs[5]
#define WEIGHT prhs[6]

/* Output Arguments */
#define PATH plhs[0]
#define NUM_STEP plhs[1]
#define NUM_NODE plhs[2]
#define TIME plhs[3]

namespace {
const std::string MAP1_PATH = "./maps/map1.txt";
const std::string MAP2_PATH = "./maps/map2.txt";
const std::string MAP3_PATH = "./maps/map3.txt";
const std::string MAP_TEST_PATH = "./maps/map_test.txt";
}  // namespace

tuple<vector<vector<double>>, int, int, double> plan(
    int map_id, double grid_size, double margin_size, double robot_x,
    double robot_y, double robot_z, double goal_x, double goal_y, double goal_z,
    bool use_dstar, double weight) {
  std::string map_path;
  vector<vector<double>> solution = {};
  int num_step;
  int num_node;
  double run_time;

  switch (map_id) {
    case 1:
      map_path = MAP1_PATH;
      break;

    case 2:
      map_path = MAP2_PATH;
      break;

    case 3:
      map_path = MAP3_PATH;
      break;

    case 4:
      map_path = MAP_TEST_PATH;
      break;

    default:
      std::cout << "Map ID not exist\n";
      break;
  }

  if (use_dstar) {
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
    run_time = solve_time.count() / 1000.0;
    solution = dstarLite.getPath();
    num_step = dstarLite.getNumStep();
    num_node = dstarLite.getNumNode();
    std::cout << "D* Lite planner takes " << run_time
              << " seconds to find solution.\n";
  } else {
    auto start = std::chrono::high_resolution_clock::now();
    CF_PLAN::Planner astar(robot_x, robot_y, robot_z, goal_x, goal_y, goal_z,
                           map_path, grid_size, margin_size, weight);
    auto stop = std::chrono::high_resolution_clock::now();
    auto construct_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "A* planner takes " << construct_time.count() / 1000.0
              << " seconds to construct.\n";

    start = chrono::high_resolution_clock::now();
    astar.plan();
    stop = chrono::high_resolution_clock::now();
    auto solve_time = chrono::duration_cast<chrono::milliseconds>(stop - start);
    run_time = solve_time.count() / 1000.0;
    solution = astar.getPath();
    num_step = astar.getNumStep();
    num_node = astar.getNumNode();
    std::cout << "A* planner takes " << run_time
              << " seconds to find solution.\n";
  }
  return make_tuple(solution, num_step, num_node, run_time);
}

/* MEX entry function */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  /* argument check */
  if (nrhs != 7) {
    mexErrMsgIdAndTxt("MATLAB:cudaAdd:inputmismatch",
                      "Input arguments must be 6!");
  }
  if (nlhs != 4) {
    mexErrMsgIdAndTxt("MATLAB:cudaAdd:outputmismatch",
                      "Output arguments must be 4!");
  }

  int map_id = (int)mxGetScalar(MAP_ID);
  double grid_size = mxGetScalar(GRID_SIZE);
  double margin_size = mxGetScalar(MARGIN_SIZE);

  double *start_ptr = mxGetPr(START);
  double *goal_ptr = mxGetPr(GOAL);

  bool use_dstar = (bool)mxGetScalar(USE_DSTAR);

  double weight = mxGetScalar(WEIGHT);

  if (mxGetM(START) != 3 || mxGetN(START) != 1) {
    mexErrMsgIdAndTxt("MATLAB:cudaAdd:sizemismatch",
                      "Input matrices must have size of 3x1!");
    return;
  }

  if (mxGetM(GOAL) != 3 || mxGetN(GOAL) != 1) {
    mexErrMsgIdAndTxt("MATLAB:cudaAdd:sizemismatch",
                      "Input matrices must have size of 3x1!");
    return;
  }

  double robot_x = start_ptr[0];
  double robot_y = start_ptr[1];
  double robot_z = start_ptr[2];

  double goal_x = goal_ptr[0];
  double goal_y = goal_ptr[1];
  double goal_z = goal_ptr[2];

  auto res = plan(map_id, grid_size, margin_size, robot_x, robot_y, robot_z,
                  goal_x, goal_y, goal_z, use_dstar, weight);
  auto solution = get<0>(res);
  int num_step = get<1>(res);
  int num_node = get<2>(res);
  double run_time = get<3>(res);

  mwSize m = solution.size();
  mwSize n = 3;
  PATH = mxCreateDoubleMatrix(m, n, mxREAL);
  double *path_ptr = mxGetPr(PATH);

  NUM_STEP = mxCreateDoubleScalar(num_step);
  NUM_NODE = mxCreateDoubleScalar(num_step);
  TIME = mxCreateDoubleScalar(run_time);

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      path_ptr[j * m + i] = solution[i][j];
    }
  }

  return;
}
