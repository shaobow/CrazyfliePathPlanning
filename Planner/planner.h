#ifndef PLANNER_H__
#define PLANNER_H__

#include "Sensor/sensor.h"

namespace CF_PLAN {

class Planner {
 private:
  // sensor for local map update
  Sensor sensor;

 public:
  Planner(/* args */);
  ~Planner();
};
}  // namespace CF_PLAN

#endif