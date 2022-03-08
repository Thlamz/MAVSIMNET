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
    	int systemId = default(255);
    	// Component ID of this GCS
        int componentId = default(255);
        // Port that MAVProxy should use
        int connectionPort = default(5505);
        // Command to run the shell used to execute SITL. 
        // You should leave this empty unless you are using some integrated environment like Cygwin
        // The commands called will be formed as: shellCommand "SITLPath -arguments"
        string shellCommand = default("");
        // Path to the sim_vehicle.py file in your SITL installation. Required
		string simulatorPath = default("");
		// Path to your MAVProxy installation
		// This is not necessary if your MAVProxy installation is already in the PATH environment variable.
		// Beware that if you are running OMNeT++ on windows on mingw your environment variables are overriden, and you should specify this.
        string MAVProxyPath = default("");
		
}
