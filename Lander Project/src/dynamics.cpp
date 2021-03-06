/* Functions to calculate the Dynamics on an object*/
#include "lander.h"
#include "lander_graphics.h"
#include <random>
#include "orbiter.h"
#include "dynamics.h"

extern lander mars_lander;


double wind(const lander &mars_lander)
{
	constexpr double average_speed = -10; //(m/s)
	std::normal_distribution<double> distribution(average_speed, 5.0);
	std::default_random_engine generator(mars_lander.timer);
	double wind_speed = distribution(generator);
	if (fabs((wind_speed - average_speed) / average_speed) < 0.3 || fabs(wind_speed-average_speed) < 5) wind_speed = average_speed;
	return wind_enabled*wind_speed;

}

double kh_tuner(const lander &mars_lander, const bool mode)
{
	if (delay_enabled || lag_enabled) return 0.017;
	// mode 1 = fuel efficiency     mode 0 = soft landing
	// double timer = 0;

	//save copy of buffer to restore afterwards
	double *temp_buffer = new double[mars_lander.throttle_buffer_length];
	for (uint i = 0; i < mars_lander.throttle_buffer_length; i++)
	{
		temp_buffer[i] = mars_lander.throttle_buffer[i];
	}

	lander virt_lander = mars_lander;
	virt_lander.set_virt_obj(true);
	double Kh_upper;
	double Kh_lower;
	if (mode)
	{
		Kh_upper = 0.2;
		Kh_lower = 0.01;
	}
	else
	{
		Kh_upper = 0.05;
		Kh_lower = 0.008;
	}
	virt_lander.Kh = (Kh_upper + Kh_lower) / 2;
	while(true)
	{
		virt_lander.numerical_dynamics();
		virt_lander.attitude_stabilization();
		virt_lander.autopilot();
		virt_lander.timer += delta_t;

		if (virt_lander.get_altitude() <= 0.5)
		{
			if (mode)
			{
				if (fabs(virt_lander.get_climb_speed()) > 1.0 || fabs(virt_lander.get_ground_speed()) > 1.0)
				{
					if(virt_lander.fuel <= 0) Kh_lower = virt_lander.Kh; //check that incase the mid value has skipped the 'landable range'
					else                      Kh_upper = virt_lander.Kh;
				}
				else Kh_lower = virt_lander.Kh;
			}
			else
			{
				if ((fabs(virt_lander.get_climb_speed()) > 1.0 || fabs(virt_lander.get_ground_speed() > 1.0)) && virt_lander.fuel <= 0.0008) Kh_lower = virt_lander.Kh;
				else Kh_upper = virt_lander.Kh;
			}

			if ((Kh_upper - Kh_lower) / Kh_upper < 0.01) break;
			//reset loop
			virt_lander = mars_lander;
			virt_lander.set_virt_obj(true);
			virt_lander.Kh = (Kh_upper + Kh_lower) / 2;
		}

		if (virt_lander.timer - simulation_time > 1500)
		{
			std::cout << "Tuner timed out\n";
			return 0.0; //add time out
		}
	}
	//reset buffer
	for (uint i = 0; i < mars_lander.throttle_buffer_length; i++)
	{
		virt_lander.throttle_buffer[i] = temp_buffer[i];
	}
	delete temp_buffer;
	temp_buffer = nullptr;

	//to ensure it does actually land return the value within the safe bounds
	if (mode)
	{
		std::cout << "Tuned Kh Value: " << Kh_lower << "\n";
		return Kh_lower;
	}
	else
	{
		std::cout << "Tuned Kh Value: " << Kh_upper << "\n";
		return Kh_upper; //to ensure it does actually land
	}
}
