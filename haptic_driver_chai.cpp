/*
 * haptic_driver_chai.cpp
 *
 *      This driver handles any haptic device defined in chai3D. any number of
 * devices can be managed by the driver. The device data for the connected
 * device are exchanged through redis keys.
 *
 *      Authors:
 * 			Margot Vulliez
 * 			Mikael Jorda
 */

#include <signal.h>

#include <Eigen/Dense>
#include <string>

#include "chai3d.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

using namespace std;
using namespace Eigen;
using namespace chai3d;

bool runloop = true;
void sighandler(int sig) { runloop = false; }

namespace {

vector<cGenericHapticDevicePtr> haptic_devices_ptr;

const string REDIS_KEY_PREFIX = "sai2::ChaiHapticDevice::device";

const string MAX_STIFFNESS_KEY_SUFFIX = "::specifications::max_stiffness";
const string MAX_DAMPING_KEY_SUFFIX = "::specifications::max_damping";
const string MAX_FORCE_KEY_SUFFIX = "::specifications::max_force";
const string COMMANDED_FORCE_KEY_SUFFIX = "::actuators::commanded_force";
const string COMMANDED_TORQUE_KEY_SUFFIX = "::actuators::commanded_torque";
const string COMMANDED_GRIPPER_FORCE_KEY_SUFFIX =
	"::actuators::commanded_force_gripper";
const string POSITION_KEY_SUFFIX = "::sensors::current_position";
const string ROTATION_KEY_SUFFIX = "::sensors::current_rotation";
const string GRIPPER_POSITION_KEY_SUFFIX =
	"::sensors::current_position_gripper";
const string LINEAR_VELOCITY_KEY_SUFFIX = "::sensors::current_trans_velocity";
const string ANGULAR_VELOCITY_KEY_SUFFIX = "::sensors::current_rot_velocity";
const string GRIPPER_VELOCITY_KEY_SUFFIX =
	"::sensors::current_gripper_velocity";
const string SENSED_FORCE_KEY_SUFFIX = "::sensors::sensed_force";
const string SENSED_TORQUE_KEY_SUFFIX = "::sensors::sensed_torque";
const string USE_GRIPPER_AS_SWITCH_KEY_SUFFIX =
	"::sensors::use_gripper_as_switch";

string createRedisKey(const string& key_suffix, int device_number) {
	return REDIS_KEY_PREFIX + to_string(device_number) + key_suffix;
}

const string HAPTIC_DEVICES_SWAP_KEY = "sai2::ChaiHapticDevice::swapDevices";
Vector2d swap_devices = Vector2d(0,0);

// Declare variable for haptic device 0
// Maximum stiffness, damping and force specifications
// vector<Vector2d> max_stiffnesses;
// vector<Vector2d> max_dampings;
// vector<Vector2d> max_forces;
// Set force and torque feedback of the haptic device
vector<Vector3d> commanded_forces;
vector<Vector3d> commanded_torques;
vector<double> commanded_force_grippers;
// Haptic device current position and rotation
vector<Vector3d> positions;
vector<Matrix3d> rotations;
vector<double> gripper_positions;
// Haptic device current velocity
vector<Vector3d> linear_velocities;
vector<Vector3d> angular_velocities;
vector<double> gripper_velocities;
// Sensed force and torque from the haptic device
vector<Vector3d> sensed_forces;
vector<Vector3d> sensed_torques;

vector<int> use_gripper_as_switch;

}  // namespace

int main() {
	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// Find and initialize connected haptic devices
	auto handler = new cHapticDeviceHandler();
	const int num_devices = handler->getNumDevices();

	if (num_devices == 0) {
		cout << "No haptic device found." << endl;
		return 0;
	}
	cout << "Found " << num_devices << " device(s) connected" << endl;

	for (int i = 0; i < num_devices; ++i) {
		cGenericHapticDevicePtr current_device_ptr;
		handler->getDevice(current_device_ptr, i);
		if (!current_device_ptr->open()) {
			for (int k = 0; k < i; k++) {
				haptic_devices_ptr[k]->close();
			}
			cout << "could not open device " << i << endl;
			return 0;
		}
		if (!current_device_ptr->calibrate()) {
			for (int k = 0; k < i; k++) {
				haptic_devices_ptr[k]->close();
			}
			cout << "could not calibrate device. If using Novint Falcon, you "
					"need to calibrate it by manually."
				 << i << endl;
			return 0;
		}

		// get device info
		cHapticDeviceInfo current_device_info;
		handler->getDeviceSpecifications(current_device_info, i);

		// if sigma 7, tell it explicitely to enable forces
		cout << "Device " << i << " is a " << current_device_info.m_modelName
			 << endl;
		if (current_device_info.m_modelName == "sigma.7") {
			cDeltaDevice* tmp_device =
				dynamic_cast<cDeltaDevice*>(current_device_ptr.get());
			tmp_device->enableForces(true);
		}

		// Send Zero force feedback to the haptic devices
		current_device_ptr->setForceAndTorqueAndGripperForce(
			cVector3d(0, 0, 0), cVector3d(0, 0, 0), 0);

		// get specifications and send them to redis
		haptic_devices_ptr.push_back(current_device_ptr);

		redis_client.setEigen(
			createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, i),
			Vector2d(current_device_info.m_maxLinearStiffness,
					 current_device_info.m_maxAngularStiffness));
		redis_client.setEigen(
			createRedisKey(MAX_DAMPING_KEY_SUFFIX, i),
			Vector2d(current_device_info.m_maxLinearDamping,
					 current_device_info.m_maxAngularDamping));
		redis_client.setEigen(createRedisKey(MAX_FORCE_KEY_SUFFIX, i),
							  Vector2d(current_device_info.m_maxLinearForce,
									   current_device_info.m_maxAngularTorque));

		// populate state
		commanded_forces.push_back(Vector3d::Zero());
		commanded_torques.push_back(Vector3d::Zero());
		commanded_force_grippers.push_back(0.0);
		positions.push_back(Vector3d::Zero());
		rotations.push_back(Matrix3d::Identity());
		gripper_positions.push_back(0.0);
		linear_velocities.push_back(Vector3d::Zero());
		angular_velocities.push_back(Vector3d::Zero());
		gripper_velocities.push_back(0.0);
		sensed_forces.push_back(Vector3d::Zero());
		sensed_torques.push_back(Vector3d::Zero());
		use_gripper_as_switch.push_back(0);

		// set commanded initial value to 0 in redis database
		redis_client.setEigen(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, i),
							  commanded_forces.at(i));
		redis_client.setEigen(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, i),
							  commanded_torques.at(i));
		redis_client.setDouble(
			createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX, i),
			commanded_force_grippers.at(i));
		redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, i),
							use_gripper_as_switch.at(i));
	}
	redis_client.setEigen(HAPTIC_DEVICES_SWAP_KEY, swap_devices);

	// prepare redis communications
	for (int i = 0; i < num_devices; ++i) {
		redis_client.addToReceiveGroup(
			createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, i),
			commanded_forces.at(i));
		redis_client.addToReceiveGroup(
			createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, i),
			commanded_torques.at(i));
		redis_client.addToReceiveGroup(
			createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX, i),
			commanded_force_grippers.at(i));
		redis_client.addToReceiveGroup(
			createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, i),
			use_gripper_as_switch.at(i));

		redis_client.addToSendGroup(createRedisKey(POSITION_KEY_SUFFIX, i),
									positions.at(i));
		redis_client.addToSendGroup(createRedisKey(ROTATION_KEY_SUFFIX, i),
									rotations.at(i));
		redis_client.addToSendGroup(
			createRedisKey(GRIPPER_POSITION_KEY_SUFFIX, i),
			gripper_positions.at(i));
		redis_client.addToSendGroup(
			createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, i),
			linear_velocities.at(i));
		redis_client.addToSendGroup(
			createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, i),
			angular_velocities.at(i));
		redis_client.addToSendGroup(
			createRedisKey(GRIPPER_VELOCITY_KEY_SUFFIX, i),
			gripper_velocities.at(i));
		redis_client.addToSendGroup(createRedisKey(SENSED_FORCE_KEY_SUFFIX, i),
									sensed_forces.at(i));
		redis_client.addToSendGroup(createRedisKey(SENSED_TORQUE_KEY_SUFFIX, i),
									sensed_torques.at(i));
	}
	redis_client.addToReceiveGroup(HAPTIC_DEVICES_SWAP_KEY, swap_devices);

	cout << "Initialization finished, starting the driver" << endl;

	// run driver loop
	Sai2Common::LoopTimer timer(1000, 1e6);
	try {
		while (runloop) {
			// wait for next scheduled loop
			timer.waitForNextLoop();

			redis_client.receiveAllFromGroup();

			if (swap_devices(0) != 0 || swap_devices(1) != 0) {
				// cout << "swapping devices" << endl;
				if (swap_devices(0) != swap_devices(1) &&
					swap_devices(0) >= 0 && swap_devices(1) >= 0 &&
					swap_devices(0) < num_devices &&
					swap_devices(1) < num_devices) {
					cout << "swapping devices " << swap_devices(0) << " and " << swap_devices(1) << endl;
					swap(haptic_devices_ptr.at((int)swap_devices(0)), haptic_devices_ptr.at((int)swap_devices(1)));
				}
				redis_client.setEigen(HAPTIC_DEVICES_SWAP_KEY, Vector2i(0, 0));
			}

			for (int i = 0; i < num_devices; ++i) {
				// set gripper switch mode
				haptic_devices_ptr.at(i)->setEnableGripperUserSwitch(
					use_gripper_as_switch.at(i));

				// send command forces to haptic devices
				haptic_devices_ptr.at(i)->setForceAndTorqueAndGripperForce(
					commanded_forces.at(i), commanded_torques.at(i),
					commanded_force_grippers.at(i));

				// get haptic device and gripper position and velocity
				cVector3d position;
				cMatrix3d rotation;
				cVector3d linear_velocity;
				cVector3d angular_velocity;
				haptic_devices_ptr.at(i)->getPosition(position);
				haptic_devices_ptr.at(i)->getRotation(rotation);
				haptic_devices_ptr.at(i)->getLinearVelocity(linear_velocity);
				haptic_devices_ptr.at(i)->getAngularVelocity(angular_velocity);
				positions.at(i) = position.eigen();
				rotations.at(i) = rotation.eigen();
				linear_velocities.at(i) = linear_velocity.eigen();
				angular_velocities.at(i) = angular_velocity.eigen();
				haptic_devices_ptr.at(i)->getGripperAngleRad(
					gripper_positions.at(i));

				// get sensed force and torque from the haptic device
				cVector3d sensed_force;
				cVector3d sensed_torque;
				haptic_devices_ptr.at(i)->getForce(sensed_force);
				haptic_devices_ptr.at(i)->getTorque(sensed_torque);
				sensed_forces.at(i) = sensed_force.eigen();
				sensed_torques.at(i) = sensed_torque.eigen();
			}

			redis_client.sendAllFromGroup();
		}
	} catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		runloop = false;
	}

	timer.stop();
	cout << "timer run statistics:" << endl;
	timer.printInfoPostRun();

	std::cout << "Closing the driver" << std::endl;

	// at the end, set all forces to 0 and close the devices
	for (int i = 0; i < num_devices; ++i) {
		redis_client.setEigen(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, i),
							  Vector3d::Zero());
		redis_client.setEigen(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, i),
							  Vector3d::Zero());
		redis_client.setDouble(
			createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX, i), 0.0);
		haptic_devices_ptr.at(i)->setForceAndTorqueAndGripperForce(
			cVector3d(0, 0, 0), cVector3d(0, 0, 0), 0.0);
		haptic_devices_ptr.at(i)->close();
	}

	return 0;
}
