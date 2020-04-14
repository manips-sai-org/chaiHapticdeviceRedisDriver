# Description:
This is a wrapper around chai3d in order tu use haptic devices through Redis for easiness of use with Sai2.0.
The driver supports one or two simultaneous haptic devices, and it has been tested with Force Dimension Omega6, Omega7 and Sigma7 haptic devices.

# Dependencies:
This project depends on 
* Cmake*: Meta compiler [brew, apt-get]
* Redis*: Redis server [brew, apt-get]
* Hiredis*: Redis minimalist client [brew, apt-get]
* JsonCpp*: JSON serialization [brew, apt-get]
* Chai3d*: [https://github.com/manips-sai-org/chai3d]

# Build instructions:
After installing all the dependencies, you can build the driver using cmake in a dedicated folder.
From the source folder type
```
mkdir build && cd build
cmake .. && make -j4
```

# Usage:
* Start the redis server if it is not already running. From a terminal window :
```
redis-server
```
* Plug and power the haptic device(s). 
* From the build folder
```
sudo ./chai_devices_redis_driver
```
If you don't want to use sudo, you'll need to give the user permission to read and write on the USB ports.

If everything went well, the device(s) will calibrate if needed and start floating.

# Communicating with other programs:
You can now control the haptic devices through redis. The driver will create the following redis keys in the database (everything is represented as a string and arrays use the JSON formatting) :

* sai2::ChaiHapticDevice::device0::specifications::max_stiffness
* sai2::ChaiHapticDevice::device0::specifications::max_damping
* sai2::ChaiHapticDevice::device0::specifications::max_force

are vectors of 2 elements, the first one corresponds to translation and the second one to rotation.

* sai2::ChaiHapticDevice::device0::sensors::current_position
* sai2::ChaiHapticDevice::device0::sensors::current_rotation
* sai2::ChaiHapticDevice::device0::sensors::current_position_gripper
* sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity
* sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity
* sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity

Correspond to the state of the haptic device

* sai2::ChaiHapticDevice::device0::sensors::sensed_force
* sai2::ChaiHapticDevice::device0::sensors::sensed_torque

Correspond to the sensed force and torques of the device

* sai2::ChaiHapticDevice::device0::actuators::commanded_force
* sai2::ChaiHapticDevice::device0::actuators::commanded_torque
* sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper

Are the commands to the device that need to be set to the redis database by the controller. They must respect the JSON formatting

* sai2::ChaiHapticDevice::swapDevices

Set this key to 1 each time you want to swap the haptic devices.
From a terminal window
```
redis-cli
set sai2::ChaiHapticDevice::swapDevices 1
exit
```

# License:
Currently pending licensing. PLEASE DO NOT DISTRIBUTE.
