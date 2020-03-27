#include "lander.h"
#include "dynamics.h"
#include "orbiter_class.h"
#include "lander_graphics.h"

void orbiter::numerical_dynamics()
{
	vector3d new_position;

	if (simulation_time == 0.0) old_position = position - delta_t*velocity;

	switch (intergrator) //switch based on intergration method chosen
	{
		case VERLET:
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
	update_members();
	return;
}

//calculates gravity on the lander returning a Force vector
vector3d orbiter::gravity() { return -(GRAVITY*MARS_MASS*mass / position.abs2())*position.norm(); }

vector3d orbiter::get_position() { return position; }

vector3d orbiter::get_old_position() { return old_position; }

vector3d orbiter::get_velocity() { return velocity; }

vector3d orbiter::get_planetary_rotation() { return planetary_rotation; }

vector3d orbiter::get_relative_velocity()  { return relative_velocity; }

double orbiter::get_altitude()   { return altitude; }

double orbiter::get_mass()       { return mass; }

void orbiter::set_position(vector3d input_pos) { position = input_pos; }

void orbiter::set_velocity(vector3d input_vel) { velocity = input_vel; }

void orbiter::set_altitude(double input_alt)   { altitude = input_alt; }

orbiter::orbiter(vector3d input_pos, vector3d input_vel, double input_mass, double input_radius)
{
	position = input_pos;
	velocity = input_vel;
	mass = input_mass;
	radius = input_radius;
	update_members();
}

orbiter::orbiter()
{
	position = vector3d{ MARS_RADIUS + 5 ,0,0 };
	velocity = vector3d{ 0,0,0 };
	mass = 0;
	radius = 0;
	update_members();
}

void orbiter::update_members()
{
	altitude           = position.abs() - MARS_RADIUS;
	acceleration       = gravity() / mass;
	planetary_rotation = std::sqrt(pow(position.x, 2) + pow(position.y, 2))*(2 * M_PI / MARS_DAY)*vector3d { -position.norm().y, position.norm().x, 0 };
	relative_velocity  = velocity - planetary_rotation;
}




//===========================================================================================================

lander::lander()
{
	autopilot_status = ORBIT_DESCENT;
	parachute_status = NOT_DEPLOYED;
	front_facing_area = M_PI*LANDER_SIZE*LANDER_SIZE;
	radius   = LANDER_SIZE;
	landed   = false;
	position = vector3d{ MARS_RADIUS + 5 ,0,0 };
	velocity = vector3d{ 0,0,0 };
	polar_velocity = vector3d{ 0,0,0 };
	virt_obj = false;
	fuel     = 1;
	throttle = 0;
	mass     = 0;
	timer = simulation_time;

	throttle_buffer = nullptr;
	throttle_buffer_length = 0;
	throttle_buffer_pointer = 0;
	last_time_lag_updated = -1.0;
	lagged_throttle = 0;
	update_members();
	return;
}

lander::~lander()
{
	autopilot(true); //remove all the pointers stored in static variables
	//to stop access violation if returning from tuning function, only delete pointer if actual used object
	if (!virt_obj)
	{
		delete throttle_buffer;
		throttle_buffer = nullptr;
	}
}

void lander::set_virt_obj(bool input) { virt_obj = input; }

void lander::set_orientation(vector3d input_orientation) { orientation = input_orientation; }

vector3d lander::get_orientation() { return orientation; }

double lander::get_climb_speed()   { return climb_speed; }

double lander::get_ground_speed()  { return ground_speed; }

vector3d lander::thrust_wrt_world(void)
// Works out thrust vector in the world reference frame, given the lander's orientation
// Works out thrust vector in the world reference frame, given the lander's orientation
{
	double m[16], k, delayed_throttle, lag = ENGINE_LAG*lag_enabled;
	vector3d a, b;

	if (timer < last_time_lag_updated) lagged_throttle = 0.0; // simulation restarted
	if (throttle < 0.0) throttle = 0.0;
	if (throttle > 1.0) throttle = 1.0;
	if (landed || (fuel == 0.0)) throttle = 0.0;

	if (timer != last_time_lag_updated)
	{

		// Delayed throttle value from the throttle history buffer
		if (throttle_buffer_length > 0 && delay_enabled)
		{
			delayed_throttle = throttle_buffer[throttle_buffer_pointer];
			throttle_buffer[throttle_buffer_pointer] = throttle;
			throttle_buffer_pointer = (throttle_buffer_pointer + 1) % throttle_buffer_length;
		}
		else delayed_throttle = throttle;

		// Lag, with time constant ENGINE_LAG
		if (lag <= 0.0) k = 0.0;
		else k = pow(exp(-1.0), delta_t / lag);
		lagged_throttle = k*lagged_throttle + (1.0 - k)*delayed_throttle;

		last_time_lag_updated = timer;
	}

	if (stabilized_attitude && (abs(stabilized_attitude_angle) < 1E-7) && !autopilot_enabled && (autopilot_status != ORBIT_DESCENT))
	{ // specific solution, avoids rounding errors in the more general calculation below
		b = lagged_throttle*MAX_THRUST*position.norm();
	}
	else
	{
		a.x = 0.0; a.y = 0.0; a.z = lagged_throttle*MAX_THRUST;
		xyz_euler_to_matrix(orientation, m);
		b.x = m[0] * a.x + m[4] * a.y + m[8] * a.z;
		b.y = m[1] * a.x + m[5] * a.y + m[9] * a.z;
		b.z = m[2] * a.x + m[6] * a.y + m[10] * a.z;
	}
	return b;
}

//calculates the drag on the lander returning a Force vector
vector3d lander::parachute_drag(void)
{
	vector3d rotation = atmosphere_rotation + wind(*this)*planetary_rotation.norm();
	return -0.5*atmospheric_density(position)*DRAG_COEF_CHUTE*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*(velocity - rotation).abs()*(velocity - rotation);
}

vector3d lander::lander_drag(void)
{
	vector3d rotation = atmosphere_rotation + wind(*this)*planetary_rotation.norm();
	return -0.5*front_facing_area*atmospheric_density(position)*(DRAG_COEF_LANDER)*(velocity - rotation).abs()*(velocity - rotation);
}

void lander::attitude_stabilization(void)
// Three-axis stabilization to ensure the lander's base is always pointing downwards
{
	vector3d up, left, out, axis;
	double m[16];

	//define axis of rotation
	if (autopilot_status == ORBIT_DESCENT && autopilot_enabled)
	{
		up = -relative_velocity.norm();
	}
	else if (abs(stabilized_attitude_angle) < 1E-5) //skip expensive calculations if aproximately 0
	{
		up = position.norm();
	}
	else
	{ //define the axis of rotation
		axis = (position.norm() ^ closeup_coords.right).norm();
		//convert to quaternion
		quat_t rotation_quat = axis_to_quat(axis, stabilized_attitude_angle);
		//create rotation matrix from quaternion, recycling m[] as it is re-assigned later
		quat_to_matrix(m, rotation_quat);
		//multiply position vector by rotation matrix
		up.x = position.norm().x*m[0] + position.norm().y*m[1] + position.norm().z*m[2]; // this is the direction we want the lander's nose to point in
		up.y = position.norm().x*m[4] + position.norm().y*m[5] + position.norm().z*m[6];
		up.z = position.norm().x*m[8] + position.norm().y*m[9] + position.norm().z*m[10];
	}

	// Set left to something perpendicular to up
	left.x = -up.y; left.y = up.x; left.z = 0.0;
	if (left.abs() < SMALL_NUM)
	{
		left.x = -up.z;
		left.y = 0.0;
		left.z = up.x;
	}

	left = left.norm();
	out = left^up;
	// Construct modelling matrix (rotation only) from these three vectors
	m[0] = out.x;  m[1] = out.y;  m[2] = out.z;  m[3] = 0.0;
	m[4] = left.x; m[5] = left.y; m[6] = left.z; m[7] = 0.0;
	m[8] = up.x;   m[9] = up.y;   m[10] = up.z;  m[11] = 0.0;
	m[12] = 0.0;   m[13] = 0.0;   m[14] = 0.0;   m[15] = 1.0;
	// Decomponse into xyz Euler angles
	orientation = matrix_to_xyz_euler(m);
	return;
}

void lander::update_members()
{
	fuel    -= delta_t * (FUEL_RATE_AT_MAX_THRUST*throttle) / FUEL_CAPACITY;
	if (fuel < 0) fuel = 0.0;

	mass     = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
	altitude = position.abs() - MARS_RADIUS;

	planetary_rotation = std::sqrt(pow(position.x, 2) + pow(position.y, 2))*(2 * M_PI / MARS_DAY)*vector3d { -position.norm().y, position.norm().x, 0 };
	relative_velocity  = velocity - planetary_rotation;
	atmosphere_rotation = planetary_rotation*(1 - altitude / EXOSPHERE);

	//note originally taken from safe_to_deploy function
	if (parachute_drag().abs() > MAX_PARACHUTE_DRAG || (velocity.abs() > MAX_PARACHUTE_SPEED && (altitude < EXOSPHERE))) safe_to_deploy_parachute = false;
	else safe_to_deploy_parachute = true;

	vector3d av_p = (old_position + position) / 2;
	climb_speed   = velocity*av_p.norm();
	ground_speed  = (relative_velocity - climb_speed*av_p.norm()).abs();
	polar_velocity.x = climb_speed;
	polar_velocity.y = (velocity - climb_speed*av_p.norm()).abs();

	if (!virt_obj) timer = simulation_time;

	if (landed || (fuel == 0.0)) throttle = 0.0;

	return;
}

void lander::autopilot(bool reset)
{
	//constants for ORBIT_DESCENT
	constexpr double ideal_ver = 0.5;
	constexpr double Kp = 1;
	double delta, error, Pout;

	//constants for TRANSFER_ORBIT and ORBIT_RE_ENTRY
	static bool *burst_complete = new bool(false);
	double direction = position.norm()*velocity.norm();
	// double transfer_impulse_time;
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

		case HOVER:
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