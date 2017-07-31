#pragma once
#ifndef DYNAMICS
#define DYNAMICS
#include "lander.h"
//Returns the drag force on the lander
vector3d lander_drag(void);

vector3d parachute_drag(void);

vector3d atmosphere_rotation();

double wind();

//returns the gravity force on the lander
vector3d gravity(const double &lander_mass);

void planetary_rotation_update();

//queries whether chute would be effective now
bool open_chute_query();

double kh_set(double kh_upper, double kh_lower);
#endif
