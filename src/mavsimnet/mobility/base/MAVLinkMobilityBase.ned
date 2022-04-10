//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package mavsimnet.mobility.base;

import inet.mobility.base.MovingMobilityBase;
//
// Base module for mobility modules that want to implement mobility using a simulated SITL instance.
// This module requires a MAVLinkManager module in your simulation. The path to this module should be specified in the
// managerModule paramter. 
//
// During the OMNeT++ simulation initialization phases this module will register a vehicle instance
// with the managerModule which will start a SITL instance and connect to it. This module will them be responsible for receiving
// telemetry and sending messages to the simulated vehicle.
//
// This is a base module and should not be used directly in a simulation. It is meant to be used as a base for other modules that
// want to implement realistic mobility using a SITL instance.  
simple MAVLinkMobilityBase extends MovingMobilityBase
{
    // TODO: Add support for simulating real vehicles
    parameters:
        @class(MAVLinkMobilityBase);
        // Path to the simulation's MAVLinkManager module in the simulation. This is required.
        string managerModule = default("mavlinkManager");
        // systemId of this vehicle instance. This is a unique identifier of the simulated vehicle instance. You should take care not to
        // repeat this ID if your simulation contains more than on MAVLinkMobility vehicle.
        int targetSystem = default(1);
        // componentId of this vehicle instance. Generally can be left as is.
        int targetComponent = default(1);
        
        // MAVLink type of vehicle that this class represents
        // COPTER=1
        // PLANE=2 
        // ROVER=4
        int vehicleType = default(1); 
        
        // Path for the parameters for this vehicle. 
        // Default parameters can be found for your vehicle type at https://github.com/ArduPilot/ardupilot/tree/master/Tools/autotest/default_params.
        // Be aware that incorrect parameters can prevent this vehicle from working. 
        string paramPath;
        
}
