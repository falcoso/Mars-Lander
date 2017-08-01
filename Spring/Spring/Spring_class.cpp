#include "Spring_class.h"
#include <iostream>
#include <vector>

spring::spring()
{
  m = 1;
  k = 1;
  x = 0;
  v = 1;
}

spring::~spring()
{
  std::cout << "Deletion complete\n";
}

void spring::dynamics(double t_max, double dt)
{
  // declare variables
  double t, a;

  // Euler integration
  for (t = 0; t <= t_max; t = t + dt) {
    // append current state to trajectories
    t_list.push_back(t);
    x_list.push_back(x);
    v_list.push_back(v);

    // calculate new position and velocity
    a = -k * x / m;
    x = x + dt * v;
    v = v + dt * a;
  }
}
