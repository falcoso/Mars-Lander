#pragma once
#ifndef DYNAMICS
#define DYNAMICS
#include "lander.h"
//Returns the drag force on the lander

double wind();

//returns the gravity force on the lander

//queries whether chute would be effective now
bool open_chute_query();

//double kh_set(double kh_upper, double kh_lower);
#endif
