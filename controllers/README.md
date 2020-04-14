# Description
These controllers implement a very simple master-slave system between two haptic devices that are supposed to be controlled through the redis driver.
The master simply gives its position and velocity as desired states to the slave (there is no haptic feedback implemented here)
The slave follows the position with a PD controller and a workspace limited to a sphere of radius 5cm.

# Usage
The controllers are built at the same time as the driver.
To use them, go to
```
cd build/controllers
```
and launch the program with the desired haptic device as an argument.
For example
```
./hapticPositionControllerSlave device0
./hapticPositionControllerMaster device1
```
will create a controler such that the first haptic device recognized by the driver is the slave and the second one is the master.
