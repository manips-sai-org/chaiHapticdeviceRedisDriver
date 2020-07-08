# create an executable
ADD_EXECUTABLE (hapticPositionControllerMaster 
	HapticPositionControllerMaster.cpp
    ../timer/LoopTimer.cpp
    ../redis/RedisClient.cpp
    )

ADD_EXECUTABLE (hapticPositionControllerSlave 
	HapticPositionControllerSlave.cpp
    ../timer/LoopTimer.cpp
    ../redis/RedisClient.cpp
    )

ADD_EXECUTABLE (localAutonomyControllerMaster 
	LocalAutonomyControllerMaster.cpp
    ../timer/LoopTimer.cpp
    ../redis/RedisClient.cpp
    )

ADD_EXECUTABLE (localAutonomyControllerSlave 
	LocalAutonomyControllerSlave.cpp
    ../timer/LoopTimer.cpp
    ../redis/RedisClient.cpp
    )

# and link the library against the executable
TARGET_LINK_LIBRARIES (hapticPositionControllerMaster ${chai_devices_redis_driver_LIBRARIES})
TARGET_LINK_LIBRARIES (hapticPositionControllerSlave ${chai_devices_redis_driver_LIBRARIES})
TARGET_LINK_LIBRARIES (localAutonomyControllerMaster ${chai_devices_redis_driver_LIBRARIES})
TARGET_LINK_LIBRARIES (localAutonomyControllerSlave ${chai_devices_redis_driver_LIBRARIES})
