#ifndef DYNAMICS
#define DYNAMICS
#include "lander.h"
#include "Orbiter class.h"
//Returns the drag force on the lander
double wind();

void dynamics_wrapper();

double kh_tuner(const lander mars_lander, const bool mode);
#endif
