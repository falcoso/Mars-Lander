#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

void verlet(double m, double k, double x, double v, double t_max, double dt, 
	std::vector<double> *t_list, std::vector<double> *v_list, std::vector<double> *x_list)
{

	// declare variables
	double t, a;

	double x2 = 0;
	double x0 = x;

	// Apply Euler integration for first step
	t = 0;

	// append current state to trajectories
	t_list->push_back(t);
	x_list->push_back(x);
	v_list->push_back(v);

	// calculate new position and velocity
	a = -k * x / m;
	x = x + dt * v;
	v = v + dt * a;

	// Apply Verlet for rest of time
	for (t = dt; t <= t_max; t = t + dt)
	{
		// append current state to trajectories
		t_list->push_back(t);
		x_list->push_back(x);
		v_list->push_back(v);

		// calculate new position and velocity
		a  = -k*x / m;
		x2 = 2 * x - x0 + dt*dt*a;
		v  = (1 / dt)*(x2 - x);

		//move x along one
		x0 = x;
		x  = x2;
	}
	std::cout << "Made it to the end of the loop" << "\n";
}


void euler(double m, double k, double x, double v, double t_max, double dt,
	std::vector<double> *t_list, std::vector<double> *v_list, std::vector<double> *x_list)
{
	// declare variables
	double t, a;

	// Euler integration
	for (t = 0; t <= t_max; t = t + dt) {
		// append current state to trajectories
		t_list->push_back(t);
		x_list->push_back(x);
		v_list->push_back(v);

		// calculate new position and velocity
		a = -k * x / m;
		x = x + dt * v;
		v = v + dt * a;
	}
}
