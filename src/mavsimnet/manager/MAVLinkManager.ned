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

package mavsimnet.manager;

//
// A module that manages connections to MAVLink vehicles. These vehicles are usually simulated vehicles in the SITL
// simulator and are connected through MAVProxy using UDP to exchange messages.
// 
// This module is not responsible for generating or interpreting any MAVLink messages, it is merely a middle-man that facilitates
// the message exchange between vehicles in the OMNeT++ simulation (represented by their mobility modules) and vehicles outside
// this simulation.
//
// This module requires you to use inet::RealTimeScheduler as a scheduler for your simulation. This can be set up by placing this line in your
// simulation's .ini:
//
// <pre>scheduler-class = "inet::RealTimeScheduler"</pre>
//
// This module's presence is <b>necessary</b> if you 
//
simple MAVLinkManager
{
    parameters:
        @class(MAVLinkManager);
        // System ID of this GCS
    	int systemId = default(235);
    	// Component ID of this GCS
        int componentId = default(235);
        // Base port for the SITL simulators
        int basePort = default(5505);
        // Path to the ArduCopter binary
		string copterSimulatorPath = default("");
		// Path to the ArduPlane binary
		string planeSimulatorPath = default("");
		// Path to the Rover binary
		string roverSimulatorPath = default("");
		
}
