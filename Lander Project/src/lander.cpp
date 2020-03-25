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
void lander::autopilot(bool reset)
{
	//constants for ORBIT_DESCENT
	constexpr double ideal_ver = 0.5;
	constexpr double Kp = 1;
	double delta, error, Pout;

	//constants for TRANSFER_ORBIT and ORBIT_RE_ENTRY
	static bool *burst_complete = new bool(false);
	double direction = position.norm()*velocity.norm();
	double transfer_impulse_time;
	static double *initial_radius = new double(position.abs());
	double target_velocity;
	static double *transfer_radius = nullptr;

	//constants for ORBITAL_INJECTION
	constexpr double Kp_v = 0.1;
	vector3d Kh_v{ 0.005, 0.0005,0 };
	vector3d Pout_v{ 0,0,0 };
	vector3d error_v{ 0,0,0, };
	vector3d delta_v;

	if (reset)
	{
		delete burst_complete;
		delete initial_radius;
		delete transfer_radius;
		burst_complete = nullptr;
		initial_radius  = nullptr;
		transfer_radius = nullptr;
		return;
	}

	//predict future positions
	vector3d temp_position = position;
	vector3d temp_velocity = velocity;
	vector3d acceleration;

	if (ENGINE_LAG != 0.0 && lag_enabled)
	{
		for (double i = 0.0; i <= ENGINE_LAG; i += delta_t)
		{
			acceleration = (gravity() + thrust_wrt_world() + lander_drag()) / mass;
			if (parachute_status == DEPLOYED)  acceleration += parachute_drag() / mass;
			velocity += delta_t*acceleration;
			position += delta_t*velocity;
		}
	}

	switch (autopilot_status)
	{
		case ORBIT_RE_ENTRY:
			//check values have not been reset
			if (burst_complete == nullptr) burst_complete = new bool(false);
			if (initial_radius == nullptr) initial_radius = new double(position.abs());
			if (transfer_radius == nullptr)
			{
				transfer_radius = new double;
				std::cout << "Input transfer radius as multiple of Mars' radius (edge of the exosphere is 1.059):" << std::endl;
				std::cin  >> *transfer_radius;
				*transfer_radius *= MARS_RADIUS;
			}
			
			//check to see if lander is at perigee or apogee of new orbit
			if (!*burst_complete)
			{
				if (velocity*closeup_coords.right > 0) stabilized_attitude_angle = (float)(acos(direction));
				else                                   stabilized_attitude_angle = (float)(-acos(direction));
				target_velocity = std::sqrt((2 * GRAVITY*MARS_MASS*(*transfer_radius)) / ((*initial_radius)*((*transfer_radius) + (*initial_radius))));

				if (*transfer_radius > *initial_radius) //going to perigee
				{
					polar_velocity.y *= -1;
					target_velocity *= -1;
				}
				else stabilized_attitude_angle += M_PI; //going to apogee

				if (polar_velocity.y > target_velocity) throttle = 1; //note possible negatives above to make equivalent to polar < target_velocity for perigee
				else
				{
					*burst_complete = true;
					throttle = 0;
				}
			}

			//burst completed
			else if (altitude < EXOSPHERE) //if within atmosphere, lander will crash so autopilot must prepare
			{
				autopilot_status = ORBIT_DESCENT;
				Kh = kh_tuner(*this, tuning_mode);
				stabilized_attitude_angle = 0;
				std::cout << "***ORBITAL RE-ENTRY COMPLETE***\n"
					<< "Fuel level:\t" << fuel * 100 << "%\n"
					<< "Descent Speed:\t" << fabs(velocity*position.norm()) << " m/s\n"
					<< "***COMMENCE ORBITAL DESCENT***" << std::endl;
				autopilot(reset);
			}
			//arrived at destination radius
			else if (fabs(climb_speed) < 5 && (position.abs() > 0.95*(*transfer_radius) && position.abs() < 1.05*(*transfer_radius)))
			{
				autopilot_status = TRANSFER_ORBIT;
				*burst_complete = false;
				std::cout << "***OBRITAL TRANSFER PHASE 1 COMPLETE***\n"
					<< "Fuel level:\t" << fuel * 100 << "%\n"
					<< "***COMMENCE ORBITAL TRANSFER PHASE 2***" << std::endl;
			}
			break;

		case TRANSFER_ORBIT:
			fuel = 1;
			target_velocity = std::sqrt(GRAVITY*MARS_MASS / *transfer_radius);
			
			if (velocity*closeup_coords.right > 0) stabilized_attitude_angle = (float)(acos(direction));
			else                                   stabilized_attitude_angle = (float)(-acos(direction));
			
			if (*transfer_radius < *initial_radius && !*burst_complete)
			{
				stabilized_attitude_angle += M_PI;
				
				if (polar_velocity.y > target_velocity) throttle = 1;
				else 
				{ 
					*burst_complete = true;
					throttle = 0; 
				}
			}
			else if (*transfer_radius > *initial_radius && !*burst_complete)
			{
				if (polar_velocity.y < target_velocity) throttle = 1;
				else *burst_complete = true;
			}

			else
			{
				autopilot_status = ORBIT_INJECTION; //tighten up orbit
				throttle = 0;
				std::cout << "***TRANSFER ORBIT PHASE 2 COMPLETED***\n"
					<< "Ground Speed:\t" << ground_speed << "\n"
					<< "***BRACE FOR ORBIT TIGHTENING***" << std::endl;
			}
			break;

		case ORBIT_DESCENT:
			/*If no parachute is available then Kh = 0.018 will land safely, if parachute is available
			a more efficient configuration is Kh = 0.03 Kp = 1 ideal_ver = 0.5
			***MOST EFFICIENT Kh WITH PARACHUTE***        ***SOFEST LANDING Kh WITH PARACHUTE***
			Scenario 1  Kh = 0.1525   Fuel Used = 7.9         Kh = 0.0104
			Scenario 3  Kh = 0.04775  Fuel Used = 35          Kh = 0.0147
			Scenario 4  Kh = 0.1525   Fuel Used = 7.9         Kh = 0.01095
			Scenario 5  Kh = 0.05143  Fuel Used = 30.1        Kh = 0.0145

			***Most EFFICIENT Kh WITHOUT PARACHUTE***
			Scenario 1  Kh = 0.04125  Fuel Used = 33.1        Kh = 0.0137
			Scenario 3  Kh = 0.01812  Fuel Used = 59.1        Kh = 0.01625
			Scenario 5  Kh = 0.01898  Fuel Used = 57.1        Kh = 0.01675
			*/
			delta = gravity().abs() / MAX_THRUST; //considering drag as part of the thrus seems to make fuel efficiency worse

			error = -(ideal_ver + Kh*altitude + climb_speed);
			Pout  = Kp*error;

			//Proportional gain control
			if (Pout <= -delta)          throttle = 0;
			else if (Pout >= 1 - delta)  throttle = 1;
			else                         throttle = delta + Pout;

			//in attitude_stabilization() up is set to velocity direction to help remove lateral motion. The specific
			//stabilised_attitude angle can be calculated with M_PI - acos(direction)

			//Determine parachute release
			if (parachute_status == NOT_DEPLOYED && altitude < 50000 && safe_to_deploy_parachute)
			{
				parachute_status = DEPLOYED;
				
				if (!virt_obj)
				{
					std::cout << "***PARACHUTE SUCCESSFULLY OPENED***\n"
						<< "Altitude:\t" << altitude << " m\n"
						<< "Descent Speed:\t" << -velocity*position.norm() << " m/s" << std::endl;
				}
			}
			else if (parachute_status == DEPLOYED && altitude < 1000) //reduce drag at lower level
			{
				if (ground_speed < fabs(1.1*wind(*this)) && fabs(wind(*this)) > 20)
				{
					parachute_status = LOST;
					
					if (!virt_obj)
					{
						std::cout << "***PARACHUTE EJECTED TO REDUCE GROUND SPEED***\n"
							<< "Altitude:\t" << altitude << "m\n"
							<< "Descent Speed:\t" << -velocity*position.norm() << "m/s" << std::endl;
					}
				}
			}
			break;

		case ORBIT_INJECTION:
			fuel = 1;
			
			if (transfer_radius == nullptr)
			{
				transfer_radius = new double;
				std::cout << "Input transfer radius as multiple of Mars' radius (Exosphere = 1.059, Aerostationary = 6.03356):" << std::endl;
				std::cin >> *transfer_radius;
				*transfer_radius *= MARS_RADIUS;
			}
			target_velocity = std::sqrt(GRAVITY*MARS_MASS / (*transfer_radius));

			error_v.x = (*transfer_radius - position.abs())*Kh_v.x - polar_velocity.x;
			error_v.y = target_velocity + (*transfer_radius - position.abs())*Kh_v.y - polar_velocity.y;

			//make sure delta is defined in the same 2D coordinate system by taking the correct components
			delta_v.x = gravity()*velocity.norm() / MAX_THRUST;
			delta_v.y = (gravity()*velocity.norm()*velocity.norm() / MAX_THRUST - delta_v.x*position.norm()).abs();
			Pout_v = Kp_v*error_v + delta_v;

			stabilized_attitude_angle = -acos(Pout_v.norm().x); //negative to work with the atmosphere rotation rather than against it for launch not from pole
			
			if (Pout_v.y < 0) stabilized_attitude_angle *= -1; //to slow down the lander if the ground speed is going too fast by pointing it in the other direction
			if (velocity*closeup_coords.right > 0 && simulation_time > 100) stabilized_attitude_angle *= -1;  //to apply correction relative to the direction of motion
			
			if (Pout_v.abs() >= 1)  throttle = 1;
			else                    throttle = Pout_v.abs();

			if (fabs((position.abs() - (*transfer_radius)) / (*transfer_radius)) < 0.001 && fabs((polar_velocity.y - target_velocity) / target_velocity) < 0.000001)
			{
				autopilot_enabled = false;
				autopilot_status  = ORBIT_RE_ENTRY; //allow further transfers
				throttle = 0;
				autopilot(true); //reset autopilot 
			}
			break;
	}
	position = temp_position;
	velocity = temp_velocity;
}

void lander::numerical_dynamics()
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
	//new position variables for verlet intergrator
	vector3d new_position;
	
	acceleration = (gravity() + thrust_wrt_world() + lander_drag()) / mass;
	if (parachute_status == DEPLOYED)  acceleration += parachute_drag() / mass;

	//so that if the simulation is reset so does the old position
	if (simulation_time == 0.0) old_position = position - delta_t*velocity;

	switch (intergrator) //switch based on intergration method chosen
	{
		case VERLET:
			//note old_position is already initialised in lander class to be an Euler timestep before
			new_position = 2 * position - old_position + delta_t*delta_t*acceleration;
			velocity = (new_position - position)/delta_t;
			//shift along positions
			old_position = position;
			position = new_position;
			break;
			
		case EULER:
			position += delta_t*velocity;
			velocity += delta_t*acceleration;
			break;
	}

	//update fuel, altitude and other derived quantities
	update_members();

	// Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
	if (stabilized_attitude) attitude_stabilization();

	// Here we can apply an autopilot to adjust the thrust, parachute and attitude
	if (autopilot_enabled)
	{
		if (Kh == 0) Kh = 0.018;
		stabilized_attitude = true;
		autopilot();
	}
	return;
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
