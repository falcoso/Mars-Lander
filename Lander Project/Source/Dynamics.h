#pragma once
#ifndef DYNAMICS
#define DYNAMICS
#include "lander.h"
//Returns the drag force on the lander
vector3d drag(void);

//returns the gravity force on the lander
vector3d gravity(const double &lander_mass);

//queries whether chute would be effective now
bool open_chute_query();
#endif