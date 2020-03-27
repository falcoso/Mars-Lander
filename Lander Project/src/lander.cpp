// Mars lander simulator
// Version 1.9
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include "dynamics.h"
#include "lander_graphics.h"
#include "orbiter_class.h"

extern lander mars_lander;

vector3d::vector3d() {x=0.0; y=0.0; z=0.0;}

vector3d::vector3d (double a, double b, double c=0.0)
{
	x = a;
	y = b;
	z = c;
}

bool vector3d::operator== (const vector3d &v) const
{
	if ((x == v.x) && (y == v.y) && (z == v.z)) return true;
	else                                        return false;
}

bool vector3d::operator!= (const vector3d &v) const
{
	if ((x != v.x) || (y != v.y) || (z != v.z)) return true;
	else                                        return false;
}

vector3d vector3d::operator+ (const vector3d &v) const { return vector3d(x + v.x, y + v.y, z + v.z); }

vector3d vector3d::operator- (const vector3d &v) const { return vector3d(x - v.x, y - v.y, z - v.z); }

vector3d operator- (const vector3d &v) { return vector3d(-v.x, -v.y, -v.z); }

vector3d& vector3d::operator+= (const vector3d &v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

vector3d& vector3d::operator-= (const vector3d &v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

vector3d vector3d::operator^ (const vector3d &v) const { return vector3d( y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }

double vector3d::operator* (const vector3d &v) const { return (x*v.x + y*v.y + z*v.z); }

vector3d operator* (const vector3d &v, const double &a) { return vector3d(v.x*a, v.y*a, v.z*a); }

vector3d operator* (const double &a, const vector3d &v) { return vector3d(v.x*a, v.y*a, v.z*a); }

vector3d& vector3d::operator*= (const double &a)
{
	x*=a;
	y*=a;
	z*=a;
	return *this;
}

vector3d vector3d::operator/ (const double &a) const { return vector3d(x/a, y/a, z/a); }

vector3d& vector3d::operator/= (const double &a)
{
	x/=a;
	y/=a;
	z/=a;
	return *this;
}

double vector3d::abs2() const { return (x*x + y*y + z*z); }

double vector3d::abs() const { return sqrt(this->abs2()); }

vector3d vector3d::norm() const
{
	double s(this->abs());
	if (s==0) return *this;
	else      return vector3d(x/s, y/s, z/s);
}

std::ostream& operator << (std::ostream &out, const vector3d &v)
{
	out << v.x << ' ' << v.y << ' ' << v.z;
	return out;
}

void initialize_simulation(void)
// Lander pose initialization - selects one of 10 possible scenarios
{
	static double aerostationary_radius = (double)pow((GRAVITY*MARS_MASS*MARS_DAY*MARS_DAY) / (4 * M_PI*M_PI), 0.333333333);
	// The parameters to set are:
	// position - in Cartesian planetary coordinate system (m)
	// velocity - in Cartesian planetary coordinate system (m/s)
	// orientation - in lander coordinate system (xyz Euler angles, degrees)
	// delta_t - the simulation time step
	// boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
	// scenario_description - a descriptive string for the help screen

	scenario_description[0] = "circular orbit";
	scenario_description[1] = "descent from 10km";
	scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
	scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
	scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
	scenario_description[5] = "descent from 200km";
	scenario_description[6] = "aerostationary orbit";
	scenario_description[7] = "descent at perfect rotation from 10km";
	scenario_description[8] = "descent at perfect rotation from 200km";
	scenario_description[9] = "";

	//reset common parameters
	delta_t = 0.1;
	mars_lander.autopilot(true);
	mars_lander.autopilot_enabled = false;
	mars_lander.parachute_status  = NOT_DEPLOYED;
	mars_lander.autopilot_status  = ORBIT_DESCENT;
	mars_lander.stabilized_attitude_angle = 0;

	switch (scenario)
	{
		case 0:
			// a circular equatorial orbit
			mars_lander.set_position (vector3d(1.2*MARS_RADIUS, 0.0, 0.0));
			mars_lander.set_velocity (vector3d(0.0, -std::sqrt(GRAVITY*MARS_MASS / (1.2*MARS_RADIUS)), 0.0));
			mars_lander.set_orientation(vector3d(0.0, 90.0, 0.0));
			mars_lander.stabilized_attitude = false;
			mars_lander.autopilot_status = ORBIT_RE_ENTRY;
			break;

		case 1:
			// a descent from rest at 10km altitude
			mars_lander.set_position(vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0));
			mars_lander.set_velocity(vector3d(0.0, 0.0, 0.0));
			mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
			mars_lander.stabilized_attitude = true;
			break;

		case 2:
			// an elliptical polar orbit
			mars_lander.set_position(vector3d(0.0, 0.0, 1.2*MARS_RADIUS));
			mars_lander.set_velocity(vector3d(3500.0, 0.0, 0.0));
			mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
			mars_lander.stabilized_attitude = false;
			mars_lander.autopilot_status = ORBIT_RE_ENTRY;
			break;

		case 3:
			// polar surface launch at escape velocity (but drag prevents escape)
			mars_lander.set_position(vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0));
			mars_lander.set_velocity(vector3d(0.0, 0.0, 5027.0));
			mars_lander.set_orientation(vector3d(0.0, 0.0, 0.0));
			mars_lander.stabilized_attitude = false;
			mars_lander.autopilot_status = ORBIT_INJECTION;
			break;

		case 4:
			// an elliptical orbit that clips the atmosphere each time round, losing energy
			mars_lander.set_position(vector3d(0.0, 0.0, MARS_RADIUS + 100000.0));
			mars_lander.set_velocity(vector3d(4000.0, 0.0, 0.0));
			mars_lander.set_orientation(vector3d(0.0, 90.0, 0.0));
			mars_lander.stabilized_attitude = false;
			break;

		case 5:
			// a descent from rest at the edge of the exosphere
			mars_lander.set_position(vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0));
			mars_lander.set_velocity(vector3d(0.0, 0.0, 0.0));
			mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
			mars_lander.stabilized_attitude = true;
			break;

		case 6:
			//orbit above a fixed point on the martian equator
			mars_lander.set_position(vector3d(aerostationary_radius, 0.0, 0.0));
			mars_lander.set_velocity(vector3d(0.0, std::sqrt(GRAVITY*MARS_MASS / aerostationary_radius), 0.0));
			mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
			mars_lander.stabilized_attitude = true;
			mars_lander.autopilot_status = ORBIT_RE_ENTRY;
			break;

		case 7:
			mars_lander.set_position(vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0));
			mars_lander.update_members();
			mars_lander.set_velocity(mars_lander.get_planetary_rotation());
			mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
			mars_lander.stabilized_attitude = true;
			break;

		case 8:
			mars_lander.set_position(vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0));
			mars_lander.update_members();
			mars_lander.set_velocity(mars_lander.get_planetary_rotation());
			mars_lander.set_orientation(vector3d(0.0, 0.0, 90.0));
			mars_lander.stabilized_attitude = true;
			break;

		case 9:
			break;
	}
	mars_lander.update_members();
}
