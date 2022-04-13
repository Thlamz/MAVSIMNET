# Getting Started
## About

MAVSIMNET is a simulation framework for the OMNeT++ discrete event simulator based on Ardupilot's Software In The Loop (SITL) simulator. It allows you to enrich OMNeT++ and INET's simulations with realistic mobility models for the mobile nodes in your network. 

It works by spawning SITL instances that provide a physical simulation of the vehicle's behaviour and connecting them to the mobility classes in your simulation. A user can transparently use these mobility modules without worrying about the details of the communication and messages being exchanged between the network simulator and the SITL instances and a developer can use the strong interface provided in the project's base mobility class to implement his own mobility modules.

## Installation

1. Clone the github repository.
> git clone https://github.com/Thlamz/MAVSIMNET/
2. Make sure you have INET installed in your workspace. If you see a directory named *inet* in your workspace you can skip this step. Note that the directory should be named *inet* and not something else.
3. Add the project to your OMNeT++ workspace. You can do this by following the steps in File > Import... > General > Existing Projects into Workspace. The root folder should be the folder you cloned in the first step.
4. Download the compiled SITL simulator for each vehicle type you want to use. Save the paths to these files as you will need them later. Binaries can be found on ardupilot's [firmware website](https://firmware.ardupilot.org/). These are the binaries for the most common platforms and supported vehicles:

| Vehicle | Windows | Linux |
| ------- | ------- | ----- |
| Copter  |  [Link](https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/) | [Link](https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/) |
|  Plane  |  [Link](https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/PlaneStable/) | [Link](https://firmware.ardupilot.org/Plane/stable/SITL_x86_64_linux_gnu/) |
|  Rover  |  [Link](https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/RoverStable/) | [Link](https://firmware.ardupilot.org/Rover/stable/SITL_x86_64_linux_gnu/) |

5. Write or download parameter files for the vehicle types you want to use. Save the paths to these files as you will need them later. We recommend donwloading the default parameter files from here:

| Vehicle | Location |
| ------- | -------- |
|  Copter | [Link](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params/copter.parm) |
|  Plane  | [Link](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params/plane.parm) |
|  Rover  | [Link](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params/rover.parm) |

To verify you have completed the installation successfuly run the randomwaypoint showcase present in the showcases folder. 

> **WARNING:** You will need to modify the .ini file for this simulation and any other that you run to include your SITL installation paths and parameter file paths for each type of vehicle that you want to use. You should have downloaded these files on steps 4 and 5. Further instructions can be found in the *Usage* section.

## Usage

The mobility modules available in this framework are instances of INET mobility modules. If you do not know what those are or how to use them you can check [INET's documentation](https://inet.omnetpp.org/docs/users-guide/index.html). There you will learn how to set up a simulation environment, populate it with nodes and [set them up with mobility modules](https://inet.omnetpp.org/docs/users-guide/index.html). After your simulation is set up there is only a couple things you need to worry about.

##### Manager

The MAVLinkManager module is responsible for starting the SITL simulation instances and communicating with them. Every MAVSIMNET simulation is **required** to have an instance of this module. It is recomended to set it up directly as a submodule of your network with the name *mavlinkManager* but you can choose any name you want, as long as you set up the *managerModule* parameter on your mobility modules. 

After placing this module in your simulation you need to set up the paths to the SITL simulators for each vehicle type supported. Currently you have to set up the *copterSimulatorPath*, *planeSimulatorPath* and *roverSimulatorPath* parameters. These are the paths to the files you have downloaded in step 4 of the installation, more specifically the path to the simulator binaries (*.elf* file on windows and extensionless file on linux). For example, if you ara on windows and have placed the copter simulator in the CopterSimulator file of the root directory of your C: drive, the parameter should be set to:

> manager.copterSimulatorPath = "C:\\\CopterSimulator\\\ArduCopter.elf"

Notice the escapes characters as windows uses back-slashes in paths. If you are not using a vehicle type you can leave the path for that vehicle's simulator as an empty string. 

Further information about this module and how it works can be found [here](/Modules/MAVLinkManager).

##### Mobility

Using the available mobility modules is as simple as setting your node's mobility module. You can do this with the following command, using the RandomWaypointMobility module as an example:

> *.client[*].mobility.typename = "MAVLinkRandomWaypointMobility"

After setting this you need to pay attention to the module's required parameters. Those can be found in the module's documentation page. Remember to pay attention if the module extends another one, as the required fields for the latter will also need to be filled. 

In general the parameters you need to fill when using any of the MAVSIMNET mobility models are:

| Parameter | Type | Description |
| --------- | ---- | ----------- |
| targetSystem | *string* | This is a unique identifier of the simulated vehicle instance. <br>You should take care not to repeat this ID if your simulation<br> contains more than on MAVLinkMobility vehicle. |
| vehicleType | *int* | MAVLink type of vehicle that this class represents <br>(COPTER=1, PLANE=2, ROVER=4) |
| paramPath | *string* | Path for the parameters for this vehicle. These are the parameters<br> you downloaded in step 5 of the installation. <br>**Do not use parameters for a different vehicle type** |
