#include "math_utils.h"

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

void invert(double m[], double mout[])
// Inverts a 4x4 OpenGL rotation matrix
{
	double zero_three, one_three, two_three;
	zero_three = -m[12] * m[0] - m[13] * m[1] - m[14] * m[2];
	one_three = -m[12] * m[4] - m[13] * m[5] - m[14] * m[6];
	two_three = -m[12] * m[8] - m[13] * m[9] - m[14] * m[10];
	mout[1] = m[4]; mout[4] = m[1]; mout[2] = m[8]; mout[8] = m[2];
	mout[6] = m[9]; mout[9] = m[6]; mout[12] = zero_three; mout[13] = one_three;
	mout[14] = two_three; mout[0] = m[0]; mout[5] = m[5]; mout[10] = m[10];
	mout[15] = 1.0; mout[3] = 0.0; mout[7] = 0.0; mout[11] = 0.0;
}

void xyz_euler_to_matrix(vector3d ang, double m[])
// Constructs a 4x4 OpenGL rotation matrix from xyz Euler angles
{
	double sin_a, sin_b, sin_g, cos_a, cos_b, cos_g;
	double ra, rb, rg;

	// Pre-calculate radian angles
	ra = ang.x*M_PI / (double)180;
	rb = ang.y*M_PI / (double)180;
	rg = ang.z*M_PI / (double)180;

	// Pre-calculate sines and cosines
	cos_a = cos(ra);
	cos_b = cos(rb);
	cos_g = cos(rg);
	sin_a = sin(ra);
	sin_b = sin(rb);
	sin_g = sin(rg);

	// Create the correct matrix coefficients
	m[0] = cos_a * cos_b;
	m[1] = sin_a * cos_b;
	m[2] = -sin_b;
	m[3] = 0.0;
	m[4] = cos_a * sin_b * sin_g - sin_a * cos_g;
	m[5] = sin_a * sin_b * sin_g + cos_a * cos_g;
	m[6] = cos_b * sin_g;
	m[7] = 0.0;
	m[8] = cos_a * sin_b * cos_g + sin_a * sin_g;
	m[9] = sin_a * sin_b * cos_g - cos_a * sin_g;
	m[10] = cos_b * cos_g;
	m[11] = 0.0;
	m[12] = 0.0;
	m[13] = 0.0;
	m[14] = 0.0;
	m[15] = 1.0;
}

vector3d matrix_to_xyz_euler(double m[])
// Decomposes a 4x4 OpenGL rotation matrix into xyz Euler angles
{
	double tmp;
	vector3d ang;

	// Catch degenerate elevation cases
	if (m[2] < -0.99999999) {
		ang.y = 90.0;
		ang.x = 0.0;
		ang.z = acos(m[8]);
		if ((sin(ang.z)>0.0) ^ (m[4]>0.0)) ang.z = -ang.z;
		ang.z *= 180.0 / M_PI;
		return ang;
	}
	if (m[2] > 0.99999999) {
		ang.y = -90.0;
		ang.x = 0.0;
		ang.z = acos(m[5]);
		if ((sin(ang.z)<0.0) ^ (m[4]>0.0)) ang.z = -ang.z;
		ang.z *= 180.0 / M_PI;
		return ang;
	}

	// Non-degenerate elevation - between -90 and +90
	ang.y = asin(-m[2]);

	// Now work out azimuth - between -180 and +180
	tmp = m[0] / cos(ang.y); // the denominator will not be zero
	if (tmp <= -1.0) ang.x = M_PI;
	else if (tmp >= 1.0) ang.x = 0.0;
	else ang.x = acos(tmp);
	if (((sin(ang.x) * cos(ang.y))>0.0) ^ ((m[1])>0.0)) ang.x = -ang.x;

	// Now work out roll - between -180 and +180
	tmp = m[10] / cos(ang.y); // the denominator will not be zero
	if (tmp <= -1.0) ang.z = M_PI;
	else if (tmp >= 1.0) ang.z = 0.0;
	else ang.z = acos(tmp);
	if (((sin(ang.z) * cos(ang.y))>0.0) ^ ((m[6])>0.0)) ang.z = -ang.z;

	// Convert to degrees
	ang.y *= 180.0 / M_PI;
	ang.x *= 180.0 / M_PI;
	ang.z *= 180.0 / M_PI;

	return ang;
}

void normalize_quat(quat_t &q)
// Normalizes a quaternion
{
	double mag;
	mag = (q.v.x*q.v.x + q.v.y*q.v.y + q.v.z*q.v.z + q.s*q.s);
	if (mag > 0.0) {
		q.v.x /= mag; q.v.y /= mag;
		q.v.z /= mag; q.s /= mag;
	}
}

quat_t axis_to_quat(vector3d a, const double phi)
// Given an axis and angle, compute quaternion
{
	quat_t q;
	q.v = a.norm() * sin(phi / 2.0);
	q.s = cos(phi / 2.0);
	return q;
}

double project_to_sphere(const double r, const double x, const double y)
// Project an x,y pair onto a sphere of radius r or a hyperbolic sheet if
// we are away from the centre of the sphere
{
	double d, t, z;

	d = sqrt(x*x + y*y);
	if (d < (r * 0.70710678118654752440)) z = sqrt(r*r - d*d);
	else { // on hyperbola
		t = r / 1.41421356237309504880;
		z = t*t / d;
	}
	return z;
}

quat_t add_quats(quat_t q1, quat_t q2)
// Given two rotations q1 and q2, calculate single equivalent quaternion
{
	quat_t s;
	vector3d t1 = q1.v * q2.s;
	vector3d t2 = q2.v * q1.s;
	vector3d t3 = q2.v ^ q1.v;
	vector3d tf = t1 + t2;
	s.v = tf + t3;
	s.s = q1.s * q2.s - q1.v * q2.v;
	normalize_quat(s);
	return s;
}

void quat_to_matrix(double m[], const quat_t q)
// Convert quaternion into a rotation matrix
{
	m[0] = 1.0 - 2.0 * (q.v.y * q.v.y + q.v.z * q.v.z);
	m[1] = 2.0 * (q.v.x * q.v.y - q.v.z * q.s);
	m[2] = 2.0 * (q.v.z * q.v.x + q.v.y * q.s);
	m[3] = 0.0;

	m[4] = 2.0 * (q.v.x * q.v.y + q.v.z * q.s);
	m[5] = 1.0 - 2.0 * (q.v.z * q.v.z + q.v.x * q.v.x);
	m[6] = 2.0 * (q.v.y * q.v.z - q.v.x * q.s);
	m[7] = 0.0;

	m[8] = 2.0 * (q.v.z * q.v.x - q.v.y * q.s);
	m[9] = 2.0 * (q.v.y * q.v.z + q.v.x * q.s);
	m[10] = 1.0 - 2.0 * (q.v.y * q.v.y + q.v.x * q.v.x);
	m[11] = 0.0;

	m[12] = 0.0; m[13] = 0.0; m[14] = 0.0; m[15] = 1.0;
}

quat_t track_quats(const double p1x, const double p1y, const double p2x, const double p2y)
// Derive quaternion from x and y mouse displacements
{
	double t, phi;
	vector3d a, p1, p2, d;
	quat_t q;

	if ((p1x == p2x) && (p1y == p2y)) {
		q.v.x = 0.0; q.v.y = 0.0; q.v.z = 0.0; q.s = 1.0;
		return q;
	}

	p1.x = p1x; p1.y = p1y;
	p1.z = project_to_sphere(0.5, p1x, p1y);
	p2.x = p2x; p2.y = p2y;
	p2.z = project_to_sphere(0.5, p2x, p2y);
	a = p2^p1; d = p1 - p2; t = d.abs();
	if (t > 1.0) t = 1.0;
	if (t < -1.0) t = -1.0;
	phi = 2.0 * asin(t);
	return axis_to_quat(a, phi);
}

void microsecond_time(unsigned long long &t)
// Returns system time in microseconds
{
#ifdef WIN32
	LARGE_INTEGER counter, frequency;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&counter);
	counter.QuadPart *= 1000000;
	t = (unsigned long long)(counter.QuadPart / frequency.QuadPart);
#else
	struct timeval tv;
	gettimeofday(&tv, NULL);
	t = (unsigned long long)tv.tv_usec + 1000000 * (unsigned long long)tv.tv_sec;
#endif
}