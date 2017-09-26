#ifndef DYNAMICS
#define DYNAMICS
#include "lander.h"
#include "Orbiter class.h"
//calcualtes the wind speed on the lander based on a normal distribution
double wind(const lander &mars_lander);

//tunes the kh value for the autopilot using and interval bisection algorithm 
//for fuel efficiency (mode = true) or soft landing (mode = false)
double kh_tuner(const lander &mars_lander, const bool mode);
#endif
