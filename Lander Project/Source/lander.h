#ifndef LANDER
#define LANDER
// Mars lander simulator
// Version 1.9
// Header file
// Gabor Csanyi and Andrew Gee, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

// Some reports suggest that Dev-C++/MinGW does not define WIN32
#if defined (__MINGW32__) && !defined (WIN32)
#define WIN32
#endif

#ifdef WIN32
#define _USE_MATH_DEFINES
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cstdlib>

// GLUT mouse wheel operations work under Linux only
#if !defined (GLUT_WHEEL_UP)
#define GLUT_WHEEL_UP 3
#define GLUT_WHEEL_DOWN 4
#endif

// Graphics constants
#define GAP 5
#define SMALL_NUM 0.0000001
#define N_RAND 20000
#define PREFERRED_WIDTH 1024
#define PREFERRED_HEIGHT 768
#define MIN_INSTRUMENT_WIDTH 1024
#define INSTRUMENT_HEIGHT 300
#define GROUND_LINE_SPACING 20.0
#define CLOSEUP_VIEW_ANGLE 30.0
#define TRANSITION_ALTITUDE 10000.0
#define TRANSITION_ALTITUDE_NO_TEXTURE 4000.0
#define TERRAIN_TEXTURE_SIZE 1024
#define INNER_DIAL_RADIUS 65.0
#define OUTER_DIAL_RADIUS 75.0
#define MAX_DELAY 160000
#define N_TRACK 1000
#define TRACK_DISTANCE_DELTA 100000.0
#define TRACK_ANGLE_DELTA 0.999
#define HEAT_FLUX_GLOW_THRESHOLD 1000000.0

// Mars constants
#define MARS_RADIUS 3386000.0 // (m)
#define MARS_MASS 6.42E23 // (kg)
#define GRAVITY 6.673E-11 // (m^3/kg/s^2)
#define MARS_DAY 88642.65 // (s)
#define EXOSPHERE 200000.0 // (m)

#define PHOBOS_RADIUS 11111.0 //m

// Lander constants
#define LANDER_SIZE 1.0 // (m)
#define UNLOADED_LANDER_MASS 100.0 // (kg)
#define FUEL_CAPACITY 100.0 // (l)
#define FUEL_RATE_AT_MAX_THRUST 0.5 // (l/s)
#define FUEL_DENSITY 1.0 // (kg/l)
// MAX_THRUST, as defined below, is 1.5 * weight of fully loaded lander at surface
#define MAX_THRUST (1.5 * (FUEL_DENSITY*FUEL_CAPACITY+UNLOADED_LANDER_MASS) * (GRAVITY*MARS_MASS/(MARS_RADIUS*MARS_RADIUS))) // (N)
#define ENGINE_LAG 0.0 // (s)
#define ENGINE_DELAY 0.0 // (s)
#define DRAG_COEF_CHUTE 2.0
#define DRAG_COEF_LANDER 1.0
#define MAX_PARACHUTE_DRAG 20000.0 // (N)
#define MAX_PARACHUTE_SPEED 500.0 // (m/s)
#define THROTTLE_GRANULARITY 20 // for manual control
#define MAX_IMPACT_GROUND_SPEED 1.0 // (m/s)
#define MAX_IMPACT_DESCENT_RATE 1.0 // (m/s)

using namespace std;

class vector3d {
  // Utility class for three-dimensional vector operations
public:
  vector3d() {x=0.0; y=0.0; z=0.0;}
  vector3d (double a, double b, double c=0.0) {x=a; y=b; z=c;}
  bool operator== (const vector3d &v) const { if ((x==v.x)&&(y==v.y)&&(z==v.z)) return true; else return false; }
  bool operator!= (const vector3d &v) const { if ((x!=v.x)||(y!=v.y)||(z!=v.z)) return true; else return false; }
  vector3d operator+ (const vector3d &v) const { return vector3d(x+v.x, y+v.y, z+v.z); }
  vector3d operator- (const vector3d &v) const { return vector3d(x-v.x, y-v.y, z-v.z); }
  friend vector3d operator- (const vector3d &v) { return vector3d(-v.x, -v.y, -v.z); }
  vector3d& operator+= (const vector3d &v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
  vector3d& operator-= (const vector3d &v) { x-=v.x; y-=v.y; z-=v.z; return *this; }
  vector3d operator^ (const vector3d &v) const { return vector3d(y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x); }
  double operator* (const vector3d &v) const { return (x*v.x + y*v.y +z*v.z); }
  friend vector3d operator* (const vector3d &v, const double &a) { return vector3d(v.x*a, v.y*a, v.z*a); }
  friend vector3d operator* (const double &a, const vector3d &v) { return vector3d(v.x*a, v.y*a, v.z*a); }
  vector3d& operator*= (const double &a) { x*=a; y*=a; z*=a; return *this; }
  vector3d operator/ (const double &a) const { return vector3d(x/a, y/a, z/a); }
  vector3d& operator/= (const double &a) { x/=a; y/=a; z/=a; return *this; }
  double abs2() const { return (x*x + y*y + z*z); }
  double abs() const { return sqrt(this->abs2()); }
  vector3d norm() const { double s(this->abs()); if (s==0) return *this; else return vector3d(x/s, y/s, z/s); }
  friend ostream& operator << (ostream &out, const vector3d &v) { out << v.x << ' ' << v.y << ' ' << v.z; return out; }
  double x, y, z;
private:
};

// Data type for recording lander's previous positions
struct track_t {
  unsigned short n;
  unsigned short p;
  vector3d pos[N_TRACK];
};

// Quaternions for orbital view transformation
struct quat_t {
  vector3d v;
  double s;
};

// Data structure for the state of the close-up view's coordinate system
struct closeup_coords_t {
  bool initialized;
  bool backwards;
  vector3d right;
};

// Enumerated data type for parachute status
enum parachute_status_t { NOT_DEPLOYED = 0, DEPLOYED = 1, LOST = 2 };
enum intergrator_t {VERLET = 0, EULER = 1};
enum autopilot_t {ORBIT_RE_ENTRY, ORBIT_DESCENT, ORBIT_INJECTION, HOVER};


extern bool wind_enabled;
extern double delta_t, simulation_time;
extern unsigned short scenario;
extern string scenario_description[];
extern closeup_coords_t closeup_coords;
extern constexpr intergrator_t intergrator = VERLET;


// Function prototypes
void initialize_simulation (void);
#endif