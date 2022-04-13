# MAVLinkManager
## Description

 A module that manages connections to MAVLink vehicles. These vehicles are simulated using SITL instances started from this module.
 Using the SimulatorPath parameters this module runs the simulator instances during the OMNeT++ simulation initialization. It them
 connects to this instances using TCP.

 This module is not responsible for generating or interpreting any MAVLink messages, it is merely a middle-man that facilitates
 the message exchange between vehicles in the OMNeT++ simulation (represented by their mobility modules) and vehicles outside
 this simulation. It is also responsible for running the SITL instances that simulate those vehicles.

 This module requires you to use inet::RealTimeScheduler as a scheduler for your simulation. This can be set up by placing this line in your
 simulation's .ini:

 > scheduler-class = "inet::RealTimeScheduler"

 This module's presence is necessary if you want to use MAVLinkMobity mobility modules for your vehicles.

## Parameters

| Name | Type | Unit | Default value | Description |
| ---- | ---- | ---- | ------------- | ----------- |
| systemId | int |  | 235 |  System ID of this GCS. |
| componentId | int |  | 235 |  Component ID of this GCS. |
| basePort | int |  | 5505 |  Base port for the SITL simulators.<br> The actual PORT the simulators will be run is basePort + (systemId * 10) where systemId is the system ID of the vehicle<br> being simulated. |
| copterSimulatorPath | string |  |  |  Path to the ArduCopter binary. Used to run simulator instances for this vehicle<br> A stable version can be downloaded from:<br> https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/ (WINDOWS)<br> https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/ (LINUX) |
| planeSimulatorPath | string |  |  |  Path to the ArduPlane binary. Used to run simulator instances for this vehicle<br> A stable version can be downloaded from:<br> https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/PlaneStable/ (WINDOWS)<br> https://firmware.ardupilot.org/Plane/stable/SITL_x86_64_linux_gnu/ (LINUX) |
| roverSimulatorPath | string |  |  | // Path to the ArduRover binary. Used to run simulator instances for this vehicle<br> A stable version can be downloaded from:<br> https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/RoverStable/ (WINDOWS)<br> https://firmware.ardupilot.org/Rover/stable/SITL_x86_64_linux_gnu/ (LINUX)<br> Path to the Rover binary. Used to run simulator instances for this vehicle |
