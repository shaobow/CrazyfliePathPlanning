#include <iostream>
#include <mex.h>
#include <vector>

#include "planner.h"

using namespace std;

// NOTES
// compile in matlab with mex main.cpp
// test with var = main(4), var==12

// not working right now, mex compiler unable to find header files in different folders

/* Input Arguments */
#define	IN               prhs[0]

/* Output Arguments */
#define	OUT              plhs[0]

vector<vector<int>> runPlanner() {
  int robot_x = 0, robot_y = -49, robot_z = 2;
  int goal_x = 60, goal_y = 170, goal_z = 5;

  CF_PLAN::Planner astar(robot_x, robot_y, robot_z, goal_x, goal_y, goal_z);
  auto start = chrono::high_resolution_clock::now();
  astar.plan();
  auto stop = chrono::high_resolution_clock::now();
  auto solve_time = chrono::duration_cast<chrono::milliseconds>(stop - start);
  // astar.printPath();
  astar.printPath_grid();
  vector<vector<int>> solution = astar.getPath();
  // std::cout << "Planner takes " << solve_time.count() / 1000.0
  //           << " seconds to find solution.\n";
  return solution;
}

void mexFunction(int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[])
{
      /* Check for proper number of arguments */
    if (nrhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }

    vector<vector<int>> solution = runPlanner();

    // test mex functionality, just multiply an input by 3
    double in = mxGetScalar(IN);
    double out = 3 * in;
    OUT = mxCreateDoubleScalar(out);

    return;

}
