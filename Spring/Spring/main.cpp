#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

//forward declare used functions
void verlet(double m, double k, double x, double v, double t_max, double dt,
	std::vector<double> *t_list, std::vector<double> *v_list, std::vector<double> *x_list);
void euler(double m, double k, double x, double v, double t_max, double dt,
	std::vector<double> *t_list, std::vector<double> *v_list, std::vector<double> *x_list);

int main() 
{

	//declare common variables
	double m, k, x, v, t_max, dt;

	// mass, spring constant, initial position and velocity
	m = 1;
	k = 1;
	x = 0;
	v = 1;

	// simulation time and timestep
	t_max = 100;
	dt	  = 0.01;

	std::vector<double> t_list, x_list, v_list;

	// choose version of intergration
	char y;
	std::cout << "Perform Euler intergral? (y/n) otherwise perform verlet.";
	std::cin >> y;
	if (y == 'y') 
	{
		euler(m, k, x, v, t_max, dt, &t_list, &x_list, &v_list);
	}
	else 
	{
		verlet(m, k, x, v, t_max, dt, &t_list, &x_list, &v_list);
	}

	/*for(auto time : t_list)
	{
		std::cout << time << "\n";
	}*/

	// write each vector to binary file
	ofstream t_out, x_out, v_out;
	t_out.open("time_array.bin", ios::out | ios::binary | ios::app);
	if (t_out)
	{
		t_out.write(reinterpret_cast<char*>(&t_list[0]), t_list.size() * sizeof(t_list[0]));
		t_out.close();
	}
	else
	{
		std::cout << "Unable to open file" << std::endl;
		return -1;
	}
	

	// Write the trajectories to file
	/*ofstream fout;
	fout.open("trajectories.txt");
	if (fout)							//file opened successfully
	{
		for (size_t i = 0; i < t_list.size(); i = i + 1)
		{
			fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << "\n";
		}
	}
	else {								// file did not open successfully
		cout << "Could not open trajectory file for writing" << endl;
	}*/
	return 0;
}