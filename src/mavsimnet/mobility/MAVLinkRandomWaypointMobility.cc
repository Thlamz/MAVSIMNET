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
        waypointRadius = par("waypointRadius");
    }
    if (stage == 1) {
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
    EV_INFO << "New Random coordinates (x,y,z) - (lat, lon, alt): (" << targetPosition.x << "," << targetPosition.y << "," << targetPosition.z << ") - (" <<
            geoCoords.latitude.get() << "," << geoCoords.longitude.get() << "," << geoCoords.altitude.get() << ")" << std::endl;

    if(vehicleType == PLANE) {
        // Using a tolerance of triple the radius to account for imprecision in the vehicle's movement
        queueInstructions(VehicleRoutines::guidedGoto(vehicleType, geoCoords.latitude.get(), geoCoords.longitude.get(), geoCoords.altitude.get(), waypointRadius * 3,
                coordinateSystem, targetSystem, targetComponent, -1, 1));
    } else {
        // Using a tolerance of triple the radius to account for imprecision in the vehicle's movement
        queueInstructions(VehicleRoutines::guidedGoto(vehicleType, geoCoords.latitude.get(), geoCoords.longitude.get(), geoCoords.altitude.get(), waypointRadius,
                coordinateSystem, targetSystem, targetComponent, -1, 1));
    }
}


void MAVLinkRandomWaypointMobility::move() {
    Enter_Method_Silent();
    if(targetPosition != Coord::NIL && getActiveCompleted() && queueSize() == 0 && !waypointChangeMessage->isScheduled()) {
        EV_INFO << "Reached waypoint. Scheduling next random waypoint" << std::endl;
        scheduleAt(simTime() + waitTime, waypointChangeMessage);
    }

    MAVLinkMobilityBase::move();
}

void MAVLinkRandomWaypointMobility::startMovement() {
    mavlink_command_long_t cmd;
    mavlink_message_t msg;
    
    // Commanding the vehicle to takeoff
    queueInstructions(VehicleRoutines::armTakeoff(systemId, componentId, vehicleType, 50, targetSystem, targetComponent, 5, 20));

    // Setting mode to guided, to prepare for random waypoint instructions
    queueInstructions(VehicleRoutines::setMode(systemId, componentId, vehicleType, VehicleRoutines::GUIDED, targetSystem, targetComponent));

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

    // Setting the waypoint radius
    // This is only necessary on the Plane vehicle because it tries to loiter around the waypoint instead of flying directly to it
    if(vehicleType == PLANE) {
        mavlink_param_set_t set_loiter_radius {
            static_cast<float>(waypointRadius),
            targetSystem,
            targetComponent,
            "WP_LOITER_RAD",
            MAV_PARAM_TYPE_INT16
        };
        mavlink_msg_param_set_encode(targetSystem, targetComponent, &msg, &set_loiter_radius);
        queueMessage(msg, TelemetryConditions::getCheckParamValue("WP_LOITER_RAD", waypointRadius, targetSystem), 15, 3);

        mavlink_param_set_t set_radius = {
            static_cast<float>(waypointRadius),
            targetSystem,
            targetComponent,
            "WP_RADIUS",
            MAV_PARAM_TYPE_INT16
        };
        mavlink_msg_param_set_encode(targetSystem, targetComponent, &msg, &set_radius);
        queueMessage(msg, TelemetryConditions::getCheckParamValue("WP_RADIUS", waypointRadius, targetSystem), 15, 3);
    }

}

void MAVLinkRandomWaypointMobility::finish() {
    MAVLinkMobilityBase::finish();
    cancelAndDelete(waypointChangeMessage);
}

}//namespace
