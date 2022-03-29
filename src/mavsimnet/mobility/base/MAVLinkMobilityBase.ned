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

simple MAVLinkMobilityBase extends MovingMobilityBase
{
    // TODO: Add support for simulating real vehicles
    parameters:
        @class(MAVLinkMobilityBase);
        string managerModule = default("mavlinkManager");
        int targetSystem = default(1);
        int targetComponent = default(1);
        
        // MAVLink type of vehicle that this class represents
        // COPTER=1
        // PLANE=2 
        // ROVER=4
        int vehicleType = default(1); 
        
}
