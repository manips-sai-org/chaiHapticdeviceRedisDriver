# Description:
This is a wrapper around chai3d in order to use haptic devices through Redis for easiness of use with SAI.
The driver supports one or two simultaneous haptic devices, and it has been tested with Force Dimension Omega6, Omega7 and Sigma7 haptic devices.
The driver should work on Ubuntu and MacOs.

# Dependencies:
This project depends on 
* Cmake: Meta compiler [brew, apt-get]
* [Chai3d](https://github.com/manips-sai-org/chai3d)
* [SaiCommon](https://github.com/manips-sai-org/SaiCommon)

If you have installed [SAI](https://github.com/manips-sai-org/SAI), you should have everything ready to build and use this driver. Otherwise, you will need to install chai3d and SaiCommon first. Go to their install instructions by clicking on the links above.

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
sudo ./chai_devices_redis_driver <optional_redis_namespace_prefix>
```
If you don't want to use sudo, you'll need to give the user permission to read and write on the USB ports.

The optional redis namespace can be provided to use a given prefix to all the redis keys used by the driver. If no namespace is provided, the default `sai` prefix will be used.

If everything went well, the device(s) will calibrate if needed and start floating.

# Communicating with other programs:
You can now control the haptic devices through redis. The redis keys used in this driver are defined in the `SaiCommon` repository in the file [`src/redis/keys/chai_haptic_devices_driver.h](https://github.com/manips-sai-org/sai-common/blob/master/src/redis/keys/chai_haptic_devices_driver.h) The driver will create the following redis keys in the database (everything is represented as a string and arrays use the JSON formatting) :

* <prefix>::chai_haptic_devices_driver::device0::specifications::max_stiffness
* <prefix>::chai_haptic_devices_driver::device0::specifications::max_damping
* <prefix>::chai_haptic_devices_driver::device0::specifications::max_force

are vectors of 2 elements, the first one corresponds to translation and the second one to rotation.

* <prefix>::chai_haptic_devices_driver::device0::sensors::current_position
* <prefix>::chai_haptic_devices_driver::device0::sensors::current_rotation
* <prefix>::chai_haptic_devices_driver::device0::sensors::current_position_gripper
* <prefix>::chai_haptic_devices_driver::device0::sensors::current_trans_velocity
* <prefix>::chai_haptic_devices_driver::device0::sensors::current_rot_velocity
* <prefix>::chai_haptic_devices_driver::device0::sensors::current_gripper_velocity

Correspond to the state of the haptic device

* <prefix>::chai_haptic_devices_driver::device0::sensors::sensed_force
* <prefix>::chai_haptic_devices_driver::device0::sensors::sensed_torque

Correspond to the sensed force and torques of the device

* <prefix>::chai_haptic_devices_driver::device0::actuators::commanded_force
* <prefix>::chai_haptic_devices_driver::device0::actuators::commanded_torque
* <prefix>::chai_haptic_devices_driver::device0::actuators::commanded_force_gripper

Are the commands to the device that need to be set to the redis database by the controller. They must respect the JSON formatting

* <prefix>::chai_haptic_devices_driver::device0::parametrization::use_gripper_as_switch
* <prefix>::chai_haptic_devices_driver::device0::sensors::switch_pressed

Are booleans represented by "1" or "0" in the redis database. The first one is to make the haptic device use its gripper as a switch (in which case, closing the gripper will count as a button press). The second one indicates the state of the haptic device button press or gripper press (when using as a switch).

When 2 haptic devices are connected, the same keys will be present for the second device, where `device0` will be replaced by `device1`. In addition, the driver exposes the following key: 

* <prefix>::chai_haptic_devices_driver::swap_devices

Set this key to 1 each time you want to swap the haptic devices 0 and 1 (for example, you want device0 to be the left hand, but because of the order of initialization, the left hand is device1 initially).
From a terminal window
```
redis-cli
set <prefix>::chai_haptic_devices_driver::swap_devices 1
exit
```

# License:
Currently pending licensing. PLEASE DO NOT DISTRIBUTE.
