# MAVLinkMobilityBase
## Description

 Base module for mobility modules that want to implement mobility using a simulated SITL instance.
 This module requires a MAVLinkManager module in your simulation. The path to this module should be specified in the
 managerModule parameter.

 During the OMNeT++ simulation initialization phases this module will register a vehicle instance
 with the managerModule which will start a SITL instance and connect to it. This module will them be responsible for receiving
 telemetry and sending messages to the simulated vehicle.

 This is a base module and should not be used directly in a simulation. It is meant to be used as a base for other modules that
 want to implement realistic mobility using a SITL instance.
## Parameters

| Name | Type | Unit | Default value | Description |
| ---- | ---- | ---- | ------------- | ----------- |
| managerModule | string |  | "mavlinkManager" |  Path to the simulation's MAVLinkManager module in the simulation. This is required. |
| targetSystem | int |  |  |  systemId of this vehicle instance. This is a unique identifier of the simulated vehicle instance.<br> You should take care not to repeat this ID if your simulation contains more than on MAVLinkMobility<br> vehicle. |
| targetComponent | int |  | 1 |  componentId of this vehicle instance. Generally can be left as is. |
| vehicleType | int |  | 1 |  MAVLink type of vehicle that this class represents<br> COPTER=1<br> PLANE=2<br> ROVER=4 |
| paramPath | string |  |  |  Path for the parameters for this vehicle.<br> Default parameters can be found for your vehicle type [here](https://github.com/ArduPilot/ardupilot/tree/master/Tools/autotest/default_params).<br> **Be aware that incorrect parameters can prevent this vehicle from working.** |
