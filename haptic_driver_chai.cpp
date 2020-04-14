/*
 * haptic_driver_chai.cpp
 *
 *      This driver handles any haptic device defined in chai3D. 1 to 2 devices can be managed by the driver.
 * 		The device data for the connected device are exchanged through redis keys. 
 * 
 *      Author: Margot Vulliez
 */

#include "redis/RedisClient.h"
#include "chai3d.h"
#include "timer/LoopTimer.h"

#include <Eigen/Dense>
#include <string>
// #include <chrono>

#include <signal.h>

using namespace std;
using namespace Eigen;
using namespace chai3d;

unsigned long long counter = 0;

bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Declaration of internal functions Chai to Eigen ////
cVector3d convertEigenToChaiVector( Eigen::Vector3d a_vec )
{
    double x = a_vec(0);
    double y = a_vec(1);
    double z = a_vec(2);
    return cVector3d(x,y,z);
}

Eigen::Vector3d convertChaiToEigenVector( cVector3d a_vec )
{
    double x = a_vec.x();
    double y = a_vec.y();
    double z = a_vec.z();
    return Eigen::Vector3d(x,y,z);
}

cMatrix3d convertEigenToChaiRotation( Eigen::Matrix3d a_mat )
{
    return cMatrix3d( a_mat(0,0), a_mat(0,1), a_mat(0,2), a_mat(1,0), a_mat(1,1), a_mat(1,2), a_mat(2,0), a_mat(2,1), a_mat(2,2) );
}

Eigen::Matrix3d convertChaiToEigenMatrix( cMatrix3d a_mat )
{
    Eigen::Matrix3d asdf;
    asdf << a_mat(0,0), a_mat(0,1), a_mat(0,2),
            a_mat(1,0), a_mat(1,1), a_mat(1,2),
            a_mat(2,0), a_mat(2,1), a_mat(2,2);

    return asdf;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// A haptic device handler
cHapticDeviceHandler* handler;
// Haptic device variables
cGenericHapticDevicePtr hapticDevice0; // a pointer to the current haptic device
cHapticDeviceInfo device_info0; // the info of the current haptic device
cGenericHapticDevicePtr hapticDevice1; // a pointer to the current haptic device
cHapticDeviceInfo device_info1; // the info of the current haptic device

// Number of connected devices
int Nb_device = 0;

vector<string> DEVICE_MAX_STIFFNESS_KEYS;
vector<string> DEVICE_MAX_DAMPING_KEYS;
vector<string> DEVICE_MAX_FORCE_KEYS;
vector<string> DEVICE_COMMANDED_FORCE_KEYS;
vector<string> DEVICE_COMMANDED_TORQUE_KEYS;
vector<string> DEVICE_COMMANDED_GRIPPER_FORCE_KEYS;
vector<string> DEVICE_POSITION_KEYS;
vector<string> DEVICE_ROTATION_KEYS;
vector<string> DEVICE_GRIPPER_POSITION_KEYS;
vector<string> DEVICE_TRANS_VELOCITY_KEYS;
vector<string> DEVICE_ROT_VELOCITY_KEYS;
vector<string> DEVICE_GRIPPER_VELOCITY_KEYS;
vector<string> DEVICE_SENSED_FORCE_KEYS;
vector<string> DEVICE_SENSED_TORQUE_KEYS;

const string HAPTIC_DEVICES_SWAP_KEY = "sai2::ChaiHapticDevice::swapDevices";

// Declare variable for haptic device 0
// Maximum stiffness, damping and force specifications
VectorXd _max_stiffness_device0 = VectorXd::Zero(2);
VectorXd _max_damping_device0 = VectorXd::Zero(2);
VectorXd _max_force_device0 = VectorXd::Zero(2);
// Set force and torque feedback of the haptic device
Vector3d _commanded_force_device0 = Vector3d::Zero();
Vector3d _commanded_torque_device0 = Vector3d::Zero();
double _commanded_force_gripper0 = 0.0;
// Haptic device current position and rotation
Vector3d _current_position_device0 = Vector3d::Zero(); 
Matrix3d _current_rotation_device0 = Matrix3d::Identity();
double _current_position_gripper_device0 = 0.0;
// Haptic device current velocity
Vector3d _current_trans_velocity_device0 = Vector3d::Zero();
Vector3d _current_rot_velocity_device0 = Vector3d::Zero();
double _current_gripper_velocity_device0 = 0.0;
// Sensed force and torque from the haptic device
Vector3d _sensed_force_device0 = Vector3d::Zero();
Vector3d _sensed_torque_device0 = Vector3d::Zero();
// Chai related variables
cVector3d _commanded_force_device0_chai = cVector3d(0.0, 0.0, 0.0);
cVector3d _commanded_torque_device0_chai = cVector3d(0.0, 0.0, 0.0);
cVector3d _current_position_device0_chai;
cMatrix3d _current_rotation_device0_chai;
cVector3d _current_trans_velocity_device0_device_chai;
cVector3d _current_rot_velocity_device0_device_chai;
cVector3d _sensed_force_device0_chai;
cVector3d _sensed_torque_device0_chai;

// Declare variable for haptic device 1
// Maximum stiffness, damping and force specifications
VectorXd _max_stiffness_device1 = VectorXd::Zero(2);
VectorXd _max_damping_device1 = VectorXd::Zero(2);
VectorXd _max_force_device1 = VectorXd::Zero(2);
// Set force and torque feedback of the haptic device
Vector3d _commanded_force_device1 = Vector3d::Zero();
Vector3d _commanded_torque_device1 = Vector3d::Zero();
double _commanded_force_gripper1 = 0.0;
// Haptic device current position and rotation
Vector3d _current_position_device1 = Vector3d::Zero(); 
Matrix3d _current_rotation_device1 = Matrix3d::Identity();
double _current_position_gripper_device1 = 0.0;
// Haptic device current velocity
Vector3d _current_trans_velocity_device1 = Vector3d::Zero();
Vector3d _current_rot_velocity_device1 = Vector3d::Zero();
double _current_gripper_velocity_device1 = 0.0;
// Sensed force and torque from the haptic device
Vector3d _sensed_force_device1 = Vector3d::Zero();
Vector3d _sensed_torque_device1 = Vector3d::Zero();
// Chai related variables
cVector3d _commanded_force_device1_chai = cVector3d(0.0, 0.0, 0.0);
cVector3d _commanded_torque_device1_chai = cVector3d(0.0, 0.0, 0.0);
cVector3d _current_position_device1_chai;
cMatrix3d _current_rotation_device1_chai;
cVector3d _current_trans_velocity_device1_device_chai;
cVector3d _current_rot_velocity_device1_device_chai;
cVector3d _sensed_force_device1_chai;
cVector3d _sensed_torque_device1_chai;

RedisClient redis_client;

int main() {
	
	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	//// Create a handler ////
	auto handler = new cHapticDeviceHandler();

 	
	//// Open an calibrate the connected devices ////
	if (!handler->getDevice(hapticDevice0, 0))
	{
		cout << "No haptic device found. " << endl;
		Nb_device = 0;
	}
	else
	{	
		if(!hapticDevice0->open())
		{
			cout << "could not open the first haptic device" << endl;
		}
		else if(!hapticDevice0->calibrate())
		{
			cout << "could not calibrate the first haptic device" << endl;
		}
		else
		{
			Nb_device = 1;
			// read info from haptic device
			handler->getDeviceSpecifications(device_info0, 0);


			//// Create device Redis keys ////
			// Maximum stiffness, damping and force specifications
			_max_stiffness_device0 << device_info0.m_maxLinearStiffness, device_info0.m_maxAngularStiffness;
			_max_damping_device0 << device_info0.m_maxLinearDamping, device_info0.m_maxAngularDamping;
			_max_force_device0 << device_info0.m_maxLinearForce, device_info0.m_maxAngularTorque;
			DEVICE_MAX_STIFFNESS_KEYS.push_back("sai2::ChaiHapticDevice::device0::specifications::max_stiffness");
			DEVICE_MAX_DAMPING_KEYS.push_back("sai2::ChaiHapticDevice::device0::specifications::max_damping");
			DEVICE_MAX_FORCE_KEYS.push_back("sai2::ChaiHapticDevice::device0::specifications::max_force");

			// Set force and torque feedback of the haptic device
			DEVICE_COMMANDED_FORCE_KEYS.push_back("sai2::ChaiHapticDevice::device0::actuators::commanded_force");
			DEVICE_COMMANDED_TORQUE_KEYS.push_back("sai2::ChaiHapticDevice::device0::actuators::commanded_torque");
			DEVICE_COMMANDED_GRIPPER_FORCE_KEYS.push_back("sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper");

			// Haptic device current position and rotation
			DEVICE_POSITION_KEYS.push_back("sai2::ChaiHapticDevice::device0::sensors::current_position");
			DEVICE_ROTATION_KEYS.push_back("sai2::ChaiHapticDevice::device0::sensors::current_rotation");
			DEVICE_GRIPPER_POSITION_KEYS.push_back("sai2::ChaiHapticDevice::device0::sensors::current_position_gripper");

			// Haptic device current velocity
			DEVICE_TRANS_VELOCITY_KEYS.push_back("sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity");
			DEVICE_ROT_VELOCITY_KEYS.push_back("sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity");
			DEVICE_GRIPPER_VELOCITY_KEYS.push_back("sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity");

			// Sensed force and torque from the haptic device
			DEVICE_SENSED_FORCE_KEYS.push_back("sai2::ChaiHapticDevice::device0::sensors::sensed_force");
			DEVICE_SENSED_TORQUE_KEYS.push_back("sai2::ChaiHapticDevice::device0::sensors::sensed_torque");
		
			if (!handler->getDevice(hapticDevice1, 1))
			{
				cout << "No second haptic device found. " << endl;
			}
			else
			{	
				if(!hapticDevice1->open())
				{
					cout << "could not open the second haptic device" << endl;
				}
				else if(!hapticDevice1->calibrate())
				{
					cout << "could not calibrate the second haptic device" << endl;
				}
				else
				{
					Nb_device = 2;
					// read info from haptic device
					handler->getDeviceSpecifications(device_info1, 1);

					//// Create device exchanged parameters and redis keys ////
					// Maximum stiffness, damping and force specifications
					_max_stiffness_device1 << device_info1.m_maxLinearStiffness, device_info1.m_maxAngularStiffness;
					_max_damping_device1 << device_info1.m_maxLinearDamping, device_info1.m_maxAngularDamping;
					_max_force_device1 << device_info1.m_maxLinearForce, device_info1.m_maxAngularTorque;
					DEVICE_MAX_STIFFNESS_KEYS.push_back("sai2::ChaiHapticDevice::device1::specifications::max_stiffness");
					DEVICE_MAX_DAMPING_KEYS.push_back("sai2::ChaiHapticDevice::device1::specifications::max_damping");
					DEVICE_MAX_FORCE_KEYS.push_back("sai2::ChaiHapticDevice::device1::specifications::max_force");

					// Set force and torque feedback of the haptic device
					DEVICE_COMMANDED_FORCE_KEYS.push_back("sai2::ChaiHapticDevice::device1::actuators::commanded_force");
					DEVICE_COMMANDED_TORQUE_KEYS.push_back("sai2::ChaiHapticDevice::device1::actuators::commanded_torque");	
					DEVICE_COMMANDED_GRIPPER_FORCE_KEYS.push_back("sai2::ChaiHapticDevice::device1::actuators::commanded_force_gripper");

					// Haptic device current position and rotation
					DEVICE_POSITION_KEYS.push_back("sai2::ChaiHapticDevice::device1::sensors::current_position");
					DEVICE_ROTATION_KEYS.push_back("sai2::ChaiHapticDevice::device1::sensors::current_rotation");
					DEVICE_GRIPPER_POSITION_KEYS.push_back("sai2::ChaiHapticDevice::device1::sensors::current_position_gripper");

					// Haptic device current velocity
					DEVICE_TRANS_VELOCITY_KEYS.push_back("sai2::ChaiHapticDevice::device1::sensors::current_trans_velocity");
					DEVICE_ROT_VELOCITY_KEYS.push_back("sai2::ChaiHapticDevice::device1::sensors::current_rot_velocity");
					DEVICE_GRIPPER_VELOCITY_KEYS.push_back("sai2::ChaiHapticDevice::device1::sensors::current_gripper_velocity");

					// Sensed force and torque from the haptic device
					DEVICE_SENSED_FORCE_KEYS.push_back("sai2::ChaiHapticDevice::device1::sensors::sensed_force");
					DEVICE_SENSED_TORQUE_KEYS.push_back("sai2::ChaiHapticDevice::device1::sensors::sensed_torque");

				}
			}
		}	
	}

	cout << "Number of device / " << Nb_device << endl;

	//// Set device in gravity compensation ////
	if (Nb_device == 1)
	{
	//// Enable force for Sigma.7 device ////
		if(device_info0.m_modelName == "sigma.7")
		{
			cDeltaDevice* tmp_device = static_cast<cDeltaDevice*>(hapticDevice0.get());
			tmp_device->enableForces(true);
		}
		// Send Zero force feedback to the haptic devices	
		hapticDevice0->setForceAndTorqueAndGripperForce(_commanded_force_device0_chai,_commanded_torque_device0_chai, _commanded_force_gripper0);
	}
	else if (Nb_device == 2)
	{
		//// Enable force for Sigma.7 device ////
		if(device_info0.m_modelName == "sigma.7")
		{
			cDeltaDevice* tmp_device = static_cast<cDeltaDevice*>(hapticDevice0.get());
			tmp_device->enableForces(true);
		}
		if(device_info1.m_modelName == "sigma.7")
		{
			cDeltaDevice* tmp_device = static_cast<cDeltaDevice*>(hapticDevice1.get());
			tmp_device->enableForces(true);
		}
		// Send Zero force feedback to the haptic devices	
		hapticDevice0->setForceAndTorqueAndGripperForce(_commanded_force_device0_chai,_commanded_torque_device0_chai, _commanded_force_gripper0);
		hapticDevice1->setForceAndTorqueAndGripperForce(_commanded_force_device1_chai,_commanded_torque_device1_chai, _commanded_force_gripper1);
	}

	//// Start redis client ////
	auto redis_client = RedisClient();
	redis_client.connect();

	//// Set initial value to Redis keys ////
	if (Nb_device == 1)
	{
		// set initial value to commanded forces
		redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0],_commanded_force_device0);
		redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0],_commanded_torque_device0);
		redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0],to_string(_commanded_force_gripper0));
		// set the device specifications to Redis keys
		redis_client.setEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0],_max_stiffness_device0);
		redis_client.setEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0],_max_damping_device0);
		redis_client.setEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0],_max_force_device0);

	}
	else if (Nb_device == 2)
	{
		redis_client.set(HAPTIC_DEVICES_SWAP_KEY, "0");
		// set initial value to commanded forces
		redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0],_commanded_force_device0);
		redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0],_commanded_torque_device0);
		redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0],to_string(_commanded_force_gripper0));

		redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[1],_commanded_force_device1);
		redis_client.setEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[1],_commanded_torque_device1);
		redis_client.set(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[1],to_string(_commanded_force_gripper1));
		// set the device specifications to Redis keys
		redis_client.setEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[0],_max_stiffness_device0);
		redis_client.setEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[0],_max_damping_device0);
		redis_client.setEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[0],_max_force_device0);
		
		redis_client.setEigenMatrixJSON(DEVICE_MAX_STIFFNESS_KEYS[1],_max_stiffness_device1);
		redis_client.setEigenMatrixJSON(DEVICE_MAX_DAMPING_KEYS[1],_max_damping_device1);
		redis_client.setEigenMatrixJSON(DEVICE_MAX_FORCE_KEYS[1],_max_force_device1);
	}

	// prepare redis callback
	redis_client.createReadCallback(0);

	redis_client.addEigenToReadCallback(0, DEVICE_COMMANDED_FORCE_KEYS[0], _commanded_force_device0);
	redis_client.addEigenToReadCallback(0, DEVICE_COMMANDED_TORQUE_KEYS[0], _commanded_torque_device0);
	redis_client.addDoubleToReadCallback(0, DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0], _commanded_force_gripper0);

	redis_client.createWriteCallback(0);

	redis_client.addEigenToWriteCallback(0, DEVICE_POSITION_KEYS[0], _current_position_device0);
	redis_client.addEigenToWriteCallback(0, DEVICE_ROTATION_KEYS[0], _current_rotation_device0);
	redis_client.addEigenToWriteCallback(0, DEVICE_TRANS_VELOCITY_KEYS[0], _current_trans_velocity_device0);
	redis_client.addEigenToWriteCallback(0, DEVICE_ROT_VELOCITY_KEYS[0], _current_rot_velocity_device0);
	redis_client.addDoubleToWriteCallback(0, DEVICE_GRIPPER_POSITION_KEYS[0], _current_position_gripper_device0);
	redis_client.addDoubleToWriteCallback(0, DEVICE_GRIPPER_VELOCITY_KEYS[0], _current_gripper_velocity_device0);

	redis_client.addEigenToWriteCallback(0, DEVICE_SENSED_FORCE_KEYS[0], _sensed_force_device0);
	redis_client.addEigenToWriteCallback(0, DEVICE_SENSED_TORQUE_KEYS[0], _sensed_torque_device0);

	if(Nb_device == 2)
	{
		redis_client.addEigenToReadCallback(0, DEVICE_COMMANDED_FORCE_KEYS[1], _commanded_force_device1);
		redis_client.addEigenToReadCallback(0, DEVICE_COMMANDED_TORQUE_KEYS[1], _commanded_torque_device1);
		redis_client.addDoubleToReadCallback(0, DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[1], _commanded_force_gripper1);

		redis_client.addEigenToWriteCallback(0, DEVICE_POSITION_KEYS[1], _current_position_device1);
		redis_client.addEigenToWriteCallback(0, DEVICE_ROTATION_KEYS[1], _current_rotation_device1);
		redis_client.addEigenToWriteCallback(0, DEVICE_TRANS_VELOCITY_KEYS[1], _current_trans_velocity_device1);
		redis_client.addEigenToWriteCallback(0, DEVICE_ROT_VELOCITY_KEYS[1], _current_rot_velocity_device1);
		redis_client.addDoubleToWriteCallback(0, DEVICE_GRIPPER_POSITION_KEYS[1], _current_position_gripper_device1);
		redis_client.addDoubleToWriteCallback(0, DEVICE_GRIPPER_VELOCITY_KEYS[1], _current_gripper_velocity_device1);

		redis_client.addEigenToWriteCallback(0, DEVICE_SENSED_FORCE_KEYS[1], _sensed_force_device1);
		redis_client.addEigenToWriteCallback(0, DEVICE_SENSED_TORQUE_KEYS[1], _sensed_torque_device1);

	}

	//// Create a timer ////
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	/////////////////////////////// cyclic ////////////////////////////////////////
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		dt = current_time - prev_time;

		redis_client.executeReadCallback(0);

		if (Nb_device == 1)
		{
			//// Get haptic device and gripper position and velocity ////
			hapticDevice0->getPosition(_current_position_device0_chai);
			hapticDevice0->getRotation(_current_rotation_device0_chai);
		 	hapticDevice0->getLinearVelocity(_current_trans_velocity_device0_device_chai);
		 	hapticDevice0->getAngularVelocity(_current_rot_velocity_device0_device_chai);
		 	hapticDevice0->getGripperAngleRad(_current_position_gripper_device0);
			_current_position_device0 = convertChaiToEigenVector(_current_position_device0_chai);
			_current_rotation_device0 = convertChaiToEigenMatrix(_current_rotation_device0_chai);
			_current_trans_velocity_device0 = convertChaiToEigenVector(_current_trans_velocity_device0_device_chai);
			_current_rot_velocity_device0 = convertChaiToEigenVector(_current_rot_velocity_device0_device_chai);
			//// Send haptic device and gripper position and velocity to Redis keys ////
			// redis_client.setEigenMatrixJSON(DEVICE_POSITION_KEYS[0],_current_position_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_ROTATION_KEYS[0],_current_rotation_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_TRANS_VELOCITY_KEYS[0],_current_trans_velocity_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_ROT_VELOCITY_KEYS[0],_current_rot_velocity_device0);
			// redis_client.set(DEVICE_GRIPPER_POSITION_KEYS[0],to_string(_current_position_gripper_device0));
			// redis_client.set(DEVICE_GRIPPER_VELOCITY_KEYS[0], to_string(_current_gripper_velocity_device0));

			//// Get sensed force and torque from the haptic device ////
			hapticDevice0->getForce(_sensed_force_device0_chai);
			hapticDevice0->getTorque(_sensed_torque_device0_chai);
			_sensed_force_device0 = convertChaiToEigenVector(_sensed_force_device0_chai);
			_sensed_torque_device0 = convertChaiToEigenVector(_sensed_torque_device0_chai);
			//// Send sensed force to redis keys /////
			// redis_client.setEigenMatrixJSON(DEVICE_SENSED_FORCE_KEYS[0],_sensed_force_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_SENSED_TORQUE_KEYS[0],_sensed_torque_device0);

			//// Read commanded force feedback from Redis ////
			// _commanded_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0]);
			// _commanded_torque_device0 = redis_client.getEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0]);
			// _commanded_force_gripper0 = stod(redis_client.get(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0]));
			
			_commanded_force_device0_chai = convertEigenToChaiVector(_commanded_force_device0);
			_commanded_torque_device0_chai = convertEigenToChaiVector(_commanded_torque_device0);

			//// Send force feedback to the haptic devices ////	
			hapticDevice0->setForceAndTorqueAndGripperForce(_commanded_force_device0_chai,_commanded_torque_device0_chai, _commanded_force_gripper0);

		}
		else if (Nb_device == 2)
		{	
			if(stoi(redis_client.get(HAPTIC_DEVICES_SWAP_KEY)) == 1)
			{
				swap(hapticDevice0, hapticDevice1);
				redis_client.set(HAPTIC_DEVICES_SWAP_KEY, "0");
			}

			//// Get haptic device and gripper position and velocity ////
			hapticDevice0->getPosition(_current_position_device0_chai);
			hapticDevice0->getRotation(_current_rotation_device0_chai);
		 	hapticDevice0->getLinearVelocity(_current_trans_velocity_device0_device_chai);
		 	hapticDevice0->getAngularVelocity(_current_rot_velocity_device0_device_chai);
		 	hapticDevice0->getGripperAngleRad(_current_position_gripper_device0);
			_current_position_device0 = convertChaiToEigenVector(_current_position_device0_chai);
			_current_rotation_device0 = convertChaiToEigenMatrix(_current_rotation_device0_chai);
			_current_trans_velocity_device0 = convertChaiToEigenVector(_current_trans_velocity_device0_device_chai);
			_current_rot_velocity_device0 = convertChaiToEigenVector(_current_rot_velocity_device0_device_chai);

			hapticDevice1->getPosition(_current_position_device1_chai);
			hapticDevice1->getRotation(_current_rotation_device1_chai);
		 	hapticDevice1->getLinearVelocity(_current_trans_velocity_device1_device_chai);
		 	hapticDevice1->getAngularVelocity(_current_rot_velocity_device1_device_chai);
		 	hapticDevice1->getGripperAngleRad(_current_position_gripper_device1);
			_current_position_device1 = convertChaiToEigenVector(_current_position_device1_chai);
			_current_rotation_device1 = convertChaiToEigenMatrix(_current_rotation_device1_chai);
			_current_trans_velocity_device1 = convertChaiToEigenVector(_current_trans_velocity_device1_device_chai);
			_current_rot_velocity_device1 = convertChaiToEigenVector(_current_rot_velocity_device1_device_chai);

			//// Send haptic device and gripper position and velocity to Redis keys ////
			// redis_client.setEigenMatrixJSON(DEVICE_POSITION_KEYS[0],_current_position_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_ROTATION_KEYS[0],_current_rotation_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_TRANS_VELOCITY_KEYS[0],_current_trans_velocity_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_ROT_VELOCITY_KEYS[0],_current_rot_velocity_device0);
			// redis_client.set(DEVICE_GRIPPER_POSITION_KEYS[0],to_string(_current_position_gripper_device0));
			// redis_client.set(DEVICE_GRIPPER_VELOCITY_KEYS[0], to_string(_current_gripper_velocity_device0));

			// redis_client.setEigenMatrixJSON(DEVICE_POSITION_KEYS[1],_current_position_device1);
			// redis_client.setEigenMatrixJSON(DEVICE_ROTATION_KEYS[1],_current_rotation_device1);
			// redis_client.setEigenMatrixJSON(DEVICE_TRANS_VELOCITY_KEYS[1],_current_trans_velocity_device1);
			// redis_client.setEigenMatrixJSON(DEVICE_ROT_VELOCITY_KEYS[1],_current_rot_velocity_device1);
			// redis_client.set(DEVICE_GRIPPER_POSITION_KEYS[1],to_string(_current_position_gripper_device1));
			// redis_client.set(DEVICE_GRIPPER_VELOCITY_KEYS[1], to_string(_current_gripper_velocity_device1));

			//// Get sensed force and torque from the haptic device ////
			hapticDevice0->getForce(_sensed_force_device0_chai);
			hapticDevice0->getTorque(_sensed_torque_device0_chai);
			_sensed_force_device0 = convertChaiToEigenVector(_sensed_force_device0_chai);
			_sensed_torque_device0 = convertChaiToEigenVector(_sensed_torque_device0_chai);

			hapticDevice1->getForce(_sensed_force_device1_chai);
			hapticDevice1->getTorque(_sensed_torque_device1_chai);
			_sensed_force_device1 = convertChaiToEigenVector(_sensed_force_device1_chai);
			_sensed_torque_device1 = convertChaiToEigenVector(_sensed_torque_device1_chai);

			//// Send sensed force to redis keys /////
			// redis_client.setEigenMatrixJSON(DEVICE_SENSED_FORCE_KEYS[0],_sensed_force_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_SENSED_TORQUE_KEYS[0],_sensed_torque_device0);
			// redis_client.setEigenMatrixJSON(DEVICE_SENSED_FORCE_KEYS[1],_sensed_force_device1);
			// redis_client.setEigenMatrixJSON(DEVICE_SENSED_TORQUE_KEYS[1],_sensed_torque_device1);

			//// Read commanded force feedback from Redis ////
			// _commanded_force_device0 = redis_client.getEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[0]);
			// _commanded_torque_device0 = redis_client.getEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[0]);
			// _commanded_force_gripper0 = stod(redis_client.get(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[0]));
			_commanded_force_device0_chai = convertEigenToChaiVector(_commanded_force_device0);
			_commanded_torque_device0_chai = convertEigenToChaiVector(_commanded_torque_device0);

			// _commanded_force_device1 = redis_client.getEigenMatrixJSON(DEVICE_COMMANDED_FORCE_KEYS[1]);
			// _commanded_torque_device1 = redis_client.getEigenMatrixJSON(DEVICE_COMMANDED_TORQUE_KEYS[1]);
			// _commanded_force_gripper1 = stod(redis_client.get(DEVICE_COMMANDED_GRIPPER_FORCE_KEYS[1]));
			_commanded_force_device1_chai = convertEigenToChaiVector(_commanded_force_device1);
			_commanded_torque_device1_chai = convertEigenToChaiVector(_commanded_torque_device1);

			//// Send force feedback to the haptic devices ////	
			hapticDevice0->setForceAndTorqueAndGripperForce(_commanded_force_device0_chai,_commanded_torque_device0_chai, _commanded_force_gripper0);
			hapticDevice1->setForceAndTorqueAndGripperForce(_commanded_force_device1_chai,_commanded_torque_device1_chai, _commanded_force_gripper1);
		
		}

		redis_client.executeWriteCallback(0);

		prev_time = current_time;
		counter++;
	}

	if (Nb_device == 1)
	{
		//// Reset haptic devices in grabity compensation ////
		_commanded_force_device0_chai.set(0.0,0.0,0.0);
		_commanded_torque_device0_chai.set(0.0,0.0,0.0);
		_commanded_force_gripper0 = 0.0;
		hapticDevice0->setForceAndTorqueAndGripperForce(_commanded_force_device0_chai,_commanded_torque_device0_chai, _commanded_force_gripper0);

		//// Close haptic devices ////
		hapticDevice0->close();
	}
	else if (Nb_device == 2)
	{		
		//// Reset haptic devices in grabity compensation ////
		_commanded_force_device0_chai.set(0.0,0.0,0.0);
		_commanded_torque_device0_chai.set(0.0,0.0,0.0);
		_commanded_force_gripper0 = 0.0;
		hapticDevice0->setForceAndTorqueAndGripperForce(_commanded_force_device0_chai,_commanded_torque_device0_chai, _commanded_force_gripper0);

		_commanded_force_device1_chai.set(0.0,0.0,0.0);
		_commanded_torque_device1_chai.set(0.0,0.0,0.0);
		_commanded_force_gripper1 = 0.0;
		hapticDevice1->setForceAndTorqueAndGripperForce(_commanded_force_device1_chai,_commanded_torque_device1_chai, _commanded_force_gripper1);

		//// Close haptic devices ////
		hapticDevice0->close();
		hapticDevice1->close();		
	}	
	return 0;
}

