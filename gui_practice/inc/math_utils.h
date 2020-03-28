#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#define N_TRACK 1000

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

#include <iostream>
#include <cmath>

class vector3d
{
	// Utility class for three-dimensional vector operations
public:
	vector3d();
	vector3d (double a, double b, double c);
	bool operator== (const vector3d &v) const;
	bool operator!= (const vector3d &v) const;
	vector3d operator+ (const vector3d &v) const;
	vector3d        operator- (const vector3d &v) const;
	friend vector3d operator- (const vector3d &v);
	vector3d& operator+= (const vector3d &v);
	vector3d& operator-= (const vector3d &v);
	vector3d operator^ (const vector3d &v) const;
	double          operator* (const vector3d &v) const;
	friend vector3d operator* (const vector3d &v, const double &a);
	friend vector3d operator* (const double &a, const vector3d &v);
	vector3d& operator*= (const double &a);
	vector3d operator/ (const double &a) const;
	vector3d& operator/= (const double &a);
	double abs2() const;
	double abs() const;
	vector3d norm() const;
	friend std::ostream& operator << (std::ostream &out, const vector3d &v);
	double x, y, z;
private:
};

// Data type for recording lander's previous positions
struct track_t
{
	unsigned short n;
	unsigned short p;
	vector3d pos[N_TRACK];
};

// Quaternions for orbital view transformation
struct quat_t
{
	vector3d v;
	double s;
};

// Function prototypes
void invert(double m[], double mout[]);
void xyz_euler_to_matrix(vector3d ang, double m[]);
vector3d matrix_to_xyz_euler(double m[]);
void normalize_quat(quat_t &q);
quat_t axis_to_quat(vector3d a, const double phi);
double project_to_sphere(const double r, const double x, const double y);
quat_t add_quats(quat_t q1, quat_t q2);
void quat_to_matrix(double m[], const quat_t q);
quat_t track_quats(const double p1x, const double p1y, const double p2x, const double p2y);
void microsecond_time(unsigned long long &t);

#endif