// position controller to work with the force interface of chai3d devices

#include "../timer/LoopTimer.h"
#include "../redis/RedisClient.h"

#include <Eigen/Dense>
#include <string>

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

// device state and desired state (to read in loop)
string DEVICE_POSITION_KEY;
string DEVICE_TRANS_VELOCITY_KEY;
string DESIRED_DEVICE_POSITION_KEY;
string DESIRED_DEVICE_LINEAR_VELOCITY_KEY;

// device command force (to write in key)
string DEVICE_COMMANDED_FORCE_KEY;

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
		DESIRED_DEVICE_POSITION_KEY = "Sai2::ChaiHapticDevice::PositionController::device1::desired_position";
		DESIRED_DEVICE_LINEAR_VELOCITY_KEY = "Sai2::ChaiHapticDevice::PositionController::device1::desired_linear_velocity";

		DEVICE_COMMANDED_FORCE_KEY = "sai2::ChaiHapticDevice::device0::actuators::commanded_force";
	}
	else if(device_name == "device1")
	{
		DEVICE_MAX_STIFFNESS_KEY = "sai2::ChaiHapticDevice::device1::specifications::max_stiffness";
		DEVICE_MAX_DAMPING_KEY = "sai2::ChaiHapticDevice::device1::specifications::max_damping";
		DEVICE_MAX_FORCE_KEY = "sai2::ChaiHapticDevice::device1::specifications::max_force";

		DEVICE_POSITION_KEY = "sai2::ChaiHapticDevice::device1::sensors::current_position";
		DEVICE_TRANS_VELOCITY_KEY = "sai2::ChaiHapticDevice::device1::sensors::current_trans_velocity";
		DESIRED_DEVICE_POSITION_KEY = "Sai2::ChaiHapticDevice::PositionController::device0::desired_position";
		DESIRED_DEVICE_LINEAR_VELOCITY_KEY = "Sai2::ChaiHapticDevice::PositionController::device0::desired_linear_velocity";

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
	auto redis_client = RedisClient();
	redis_client.connect();

	// setup position controller
	// const double workspace_radius = 0.05;
	// double max_force = redis_client.getEigenMatrixJSON(DEVICE_MAX_FORCE_KEY)(0);

	Vector3d device_command_force = Vector3d::Zero();

	Vector3d device_desired_position = Vector3d::Zero();
	Vector3d device_desired_linear_velocity = Vector3d::Zero();

	// redis_client.setEigenMatrixJSON(DESIRED_DEVICE_POSITION_KEY, device_desired_position);
	// redis_client.setEigenMatrixJSON(DESIRED_DEVICE_LINEAR_VELOCITY_KEY, device_desired_linear_velocity);

	Vector3d device_position = redis_client.getEigenMatrixJSON(DEVICE_POSITION_KEY);
	Vector3d device_velocity = redis_client.getEigenMatrixJSON(DEVICE_TRANS_VELOCITY_KEY);

	// double kp = 0.07 * redis_client.getEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEY)(0);
	// double kv = 0.55 * redis_client.getEigenMatrixJSON(DEVICE_MAX_DAMPING_KEY)(0);

	// setup redis reads and writes
	redis_client.createReadCallback(0);

	redis_client.addEigenToReadCallback(0, DEVICE_POSITION_KEY, device_position);
	redis_client.addEigenToReadCallback(0, DEVICE_TRANS_VELOCITY_KEY, device_velocity);

	redis_client.createWriteCallback(0);
	redis_client.addEigenToWriteCallback(0, DESIRED_DEVICE_POSITION_KEY, device_desired_position);
	redis_client.addEigenToWriteCallback(0, DESIRED_DEVICE_LINEAR_VELOCITY_KEY, device_desired_linear_velocity);

	//// Create a timer ////
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while (runloop) 
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		dt = current_time - prev_time;

		redis_client.executeReadCallback(0);

		// transmit desired position and velocity
		device_desired_position = device_position;
		device_desired_linear_velocity = device_velocity;

		redis_client.executeWriteCallback(0);

	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
