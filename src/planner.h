#ifndef PLANNER_H__
#define PLANNER_H__

#include "sensor.h"

namespace CF_PLAN {

class Planner {
 private:
  // sensor for local map update
  Sensor sensor;

  // check if successor state is valid
  bool isValid(const Coord& robot_state);

 public:
  Planner(/* args */);
  ~Planner();
};
}  // namespace CF_PLAN

#endif