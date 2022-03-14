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

#include "MAVLinkRandomWaypointMobility.h"
#include "mavsimnet/utils/TelemetryConditions.h"
#include "mavsimnet/utils/VehicleRoutines.h"

using namespace omnetpp;
using namespace inet;

namespace mavsimnet {

Define_Module(MAVLinkRandomWaypointMobility);

void MAVLinkRandomWaypointMobility::initialize(int stage)
{
    MAVLinkMobilityBase::initialize(stage);
    if (stage == 0) {
        speed = par("speed");
        waitTime = par("waitTime");
        startMovement();
        setTargetPosition();
    }
}

void MAVLinkRandomWaypointMobility::handleMessage(cMessage *msg)
{
    if(msg->isSelfMessage() && strcmp(msg->getName(), "waypointChangeMessage") == 0) {
        setTargetPosition();
    }
    MAVLinkMobilityBase::handleMessage(msg);
}

void MAVLinkRandomWaypointMobility::setTargetPosition(){
    targetPosition = getRandomPosition();
    GeoCoord geoCoords = coordinateSystem->computeGeographicCoordinate(targetPosition);
    EV_INFO << "Random coordinates (x,y,z) - (lat, lon, alt): (" << targetPosition.x << "," << targetPosition.y << "," << targetPosition.z << ") - (" <<
            geoCoords.latitude.get() << "," << geoCoords.longitude.get() << "," << geoCoords.altitude.get() << ")" << std::endl;

    queueInstructions(VehicleRoutines::guidedGoto(vehicleType, geoCoords.latitude.get(), geoCoords.longitude.get(), geoCoords.altitude.get(), targetSystem, targetComponent));
}


void MAVLinkRandomWaypointMobility::move() {
    if(targetPosition != Coord::NIL && targetPosition.distance(currentPosition) <= 5 && !waypointChangeMessage->isScheduled()) {
        scheduleAt(simTime() + waitTime, waypointChangeMessage);
    }

    MAVLinkMobilityBase::move();
}

void MAVLinkRandomWaypointMobility::startMovement() {
    mavlink_command_long_t cmd;
    mavlink_message_t msg;
    
    // Commanding the vehicle to takeoff
    queueInstructions(VehicleRoutines::armTakeoff(vehicleType, 50, targetSystem, targetComponent));

    // Setting mode to guided, to prepare for random waypoint instructions
    queueInstructions(VehicleRoutines::setMode(vehicleType, VehicleRoutines::GUIDED, targetSystem, targetComponent));

    // Setting vehicle's speed
    cmd = {};
    cmd.command = MAV_CMD_DO_CHANGE_SPEED;
    cmd.confirmation = 0;
    cmd.param1 = 0;
    cmd.param2 = speed;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    queueMessage(msg, TelemetryConditions::getCheckCmdAck(targetSystem, targetComponent, MAV_CMD_DO_CHANGE_SPEED, targetSystem), 15, 3);

}

MAVLinkRandomWaypointMobility::~MAVLinkRandomWaypointMobility() {
    cancelAndDelete(waypointChangeMessage);
}

}//namespace
