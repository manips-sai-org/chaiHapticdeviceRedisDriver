// position controller to work with the force interface of chai3d devices

#include "../timer/LoopTimer.h"
#include "../redis/RedisClient.h"

#include <Eigen/Dense>
#include <string>
#include <queue>
#include <fstream>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

// redis keys
// device specifications (to read once)
string DEVICE_MAX_STIFFNESS_KEY;
string DEVICE_MAX_DAMPING_KEY;
string DEVICE_MAX_FORCE_KEY;

// device state
string DEVICE_POSITION_KEY;
string DEVICE_TRANS_VELOCITY_KEY;

// proxy to read and force space direction
string MASTER_PROXY_TO_READ_KEY = "sai2::ChaiHapticDevice::local_autonomy_controller::master_proxy";
string SIGMA_FORCE_KEY = "sai2::ChaiHapticDevice::local_autonomy_controller::sigma_force";

// proxy to write and max velocity for interpolation
string SLAVE_PROXY_TO_WRITE_KEY = "sai2::ChaiHapticDevice::local_autonomy_controller::slave_proxy";
string INTERPOLATION_VELOCITY_KEY = "sai2::ChaiHapticDevice::local_autonomy_controller::velocity_interpolation";

// device command force
string DEVICE_COMMANDED_FORCE_KEY;

// thread run functions
void communication();

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

// paramters
const double control_frequency = 1000.0;
const double communication_frequency = 50.0;
const double introduced_delay_ms = 500.0;

Matrix3d sigma_force = Matrix3d::Zero();
Vector3d master_proxy = Vector3d::Zero();
Vector3d slave_proxy = Vector3d::Zero();
Vector3d velocity_interpolation = Vector3d::Zero();

Matrix3d delayed_sigma_force = Matrix3d::Zero();
Vector3d delayed_master_proxy = Vector3d::Zero();
Vector3d delayed_slave_proxy = Vector3d::Zero();
Vector3d delayed_velocity_interpolation = Vector3d::Zero();

// const string remote_ip = "10.0.0.244";
const string remote_ip = "127.0.0.1";

int main(int argc, char** argv) {

	if(argc < 2)
	{
		cout << "Please enter the device name as an argument ('device0' or 'device1')" << endl;
		return -1;
	}
	string device_name = argv[1];

	if(device_name == "device0")
	{
		DEVICE_MAX_STIFFNESS_KEY = "sai2::ChaiHapticDevice::device0::specifications::max_stiffness";
		DEVICE_MAX_DAMPING_KEY = "sai2::ChaiHapticDevice::device0::specifications::max_damping";
		DEVICE_MAX_FORCE_KEY = "sai2::ChaiHapticDevice::device0::specifications::max_force";

		DEVICE_POSITION_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_position";
		DEVICE_TRANS_VELOCITY_KEY = "sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity";

		DEVICE_COMMANDED_FORCE_KEY = "sai2::ChaiHapticDevice::device0::actuators::commanded_force";
	}
	else if(device_name == "device1")
	{
		DEVICE_MAX_STIFFNESS_KEY = "sai2::ChaiHapticDevice::device1::specifications::max_stiffness";
		DEVICE_MAX_DAMPING_KEY = "sai2::ChaiHapticDevice::device1::specifications::max_damping";
		DEVICE_MAX_FORCE_KEY = "sai2::ChaiHapticDevice::device1::specifications::max_force";

		DEVICE_POSITION_KEY = "sai2::ChaiHapticDevice::device1::sensors::current_position";
		DEVICE_TRANS_VELOCITY_KEY = "sai2::ChaiHapticDevice::device1::sensors::current_trans_velocity";

		DEVICE_COMMANDED_FORCE_KEY = "sai2::ChaiHapticDevice::device1::actuators::commanded_force";		     
	}
	else
	{
		cout << "argument must be 'device0' or 'device1'" << endl;
		return -1;       
	}

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// start redis client
	auto redis_client_local = RedisClient();
	redis_client_local.connect();


	// setup position controller
	// const double workspace_radius = 0.05;
	double max_force = redis_client_local.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEY)(0);
	double kp = 0.7 * redis_client_local.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEY)(0);
	double kv = 0.55 * redis_client_local.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEY)(0);

	Vector3d device_position = Vector3d::Zero();
	Vector3d device_velocity = Vector3d::Zero();
	Vector3d device_command_force = Vector3d::Zero();

	// setup computation of mean velocity
	int buffer_size = control_frequency/communication_frequency;
	double mean_normalization_coeff = communication_frequency/control_frequency;
	int buffer_counter = 0;
	queue<Vector3d> master_velocity_buffer;

	// setup redis reads and writes
	redis_client_local.createReadCallback(0);
	redis_client_local.addEigenToReadCallback(0, DEVICE_POSITION_KEY, device_position);
	redis_client_local.addEigenToReadCallback(0, DEVICE_TRANS_VELOCITY_KEY, device_velocity);

	redis_client_local.createWriteCallback(0);
	redis_client_local.addEigenToWriteCallback(0, DEVICE_COMMANDED_FORCE_KEY, device_command_force);

	// prepare logging
	string file_name = "data_logging_master_" + currentDateTime();
	ofstream logging_file(file_name);
	logging_file << "master_position[3]\tmaster_velocity[3]\treceived_master_proxy[3]\tsent_slave_proxy[3]\n";

	// communication thread
	thread communication_thread(communication);

	//// Create a timer ////
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(control_frequency); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	unsigned long long controller_counter = 0;

	while (runloop) 
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		dt = current_time - prev_time;

		redis_client_local.executeReadCallback(0);
		slave_proxy = device_position;

		// compute average velocity
		master_velocity_buffer.push(device_velocity);
		velocity_interpolation += device_velocity*mean_normalization_coeff;

		if(buffer_counter > buffer_size)
		{
			velocity_interpolation -= master_velocity_buffer.front()*mean_normalization_coeff;

			master_velocity_buffer.pop();

			buffer_counter--;
		}
		buffer_counter++;

		// compute command force
		device_command_force = delayed_sigma_force * (-kp * (device_position - delayed_master_proxy) - kv*device_velocity);
		if(device_command_force.norm() > max_force)
		{
			device_command_force *= max_force / device_command_force.norm();
		}

		redis_client_local.executeWriteCallback(0);

		// logging
		logging_file << device_position.transpose() << '\t';
		logging_file << device_velocity.transpose() << '\t';
		logging_file << delayed_master_proxy.transpose() << '\t';
		logging_file << slave_proxy.transpose() << '\n';

		controller_counter++;

	}

	device_command_force.setZero();
	redis_client_local.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEY, device_command_force);

	communication_thread.join();

	logging_file.close();

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}

void communication()
{

	queue<Vector3d> master_proxy_buffer;
	queue<Vector3d> slave_proxy_buffer;
	queue<Vector3d> velocity_interpolation_buffer;
	queue<Matrix3d> sigma_force_buffer;

	auto redis_client_comm = RedisClient();
	redis_client_comm.connect(remote_ip);

	redis_client_comm.setEigenMatrixJSON(MASTER_PROXY_TO_READ_KEY, master_proxy);
	redis_client_comm.setEigenMatrixJSON(SIGMA_FORCE_KEY, sigma_force);

	// setup redis calls
	redis_client_comm.createReadCallback(1);
	redis_client_comm.addEigenToReadCallback(1, MASTER_PROXY_TO_READ_KEY, master_proxy);
	redis_client_comm.addEigenToReadCallback(1, SIGMA_FORCE_KEY, sigma_force);

	redis_client_comm.createWriteCallback(1);
	redis_client_comm.addEigenToWriteCallback(1, SLAVE_PROXY_TO_WRITE_KEY, delayed_slave_proxy);
	redis_client_comm.addEigenToWriteCallback(1, INTERPOLATION_VELOCITY_KEY, delayed_velocity_interpolation);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(communication_frequency); //Compiler en mode release
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	unsigned long long communication_counter = 0;
	const int communication_delay_ncycles = introduced_delay_ms / 1000.0 * communication_frequency;
	
	while(runloop)
	{
		timer.waitForNextLoop();

		redis_client_comm.executeReadCallback(1);

		// introduce delay if required
		if(communication_delay_ncycles == 0)
		{
			delayed_master_proxy = master_proxy;
			delayed_slave_proxy = slave_proxy;
			delayed_velocity_interpolation = velocity_interpolation;
			delayed_sigma_force = sigma_force;
		}
		else
		{
			master_proxy_buffer.push(master_proxy);
			slave_proxy_buffer.push(slave_proxy);
			velocity_interpolation_buffer.push(velocity_interpolation);
			sigma_force_buffer.push(sigma_force);

			if(communication_counter > communication_delay_ncycles)
			{
				delayed_master_proxy = master_proxy_buffer.front();
				delayed_slave_proxy = slave_proxy_buffer.front();
				delayed_velocity_interpolation = velocity_interpolation_buffer.front();
				delayed_sigma_force = sigma_force_buffer.front();

				master_proxy_buffer.pop();
				slave_proxy_buffer.pop();
				velocity_interpolation_buffer.pop();
				sigma_force_buffer.pop();

				communication_counter--;
			}

		}

		redis_client_comm.executeWriteCallback(1);

		communication_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Communication Loop run time  : " << end_time << " seconds\n";
	std::cout << "Communication Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Communication Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}
