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

using namespace omnetpp;
using namespace inet;

namespace mavsimnet {

Define_Module(MAVLinkRandomWaypointMobility);

void MAVLinkRandomWaypointMobility::initialize(int stage)
{
    if (stage == 0) {
        speed = par("speed");
        waitTime = par("waitTime");
        startMovement();
        setTargetPosition();
    }
    MAVLinkMobilityBase::initialize(stage);
}

void MAVLinkRandomWaypointMobility::handleMessage(cMessage *msg)
{
    if(msg->isSelfMessage() && strcmp(msg->getName(), "waypointChangeMessage") == 0) {
        setTargetPosition();
    }
    MAVLinkMobilityBase::handleMessage(msg);
}

void MAVLinkRandomWaypointMobility::setTargetPosition(){
    Coord targetPosition = getRandomPosition();
    double distance = lastPosition.distance(targetPosition);

    GeoCoord geoCoords = coordinateSystem->computeGeographicCoordinate(targetPosition);

    mavlink_set_position_target_global_int_t position_command;
    position_command.lat_int = geoCoords.latitude.get() * (1e7);
    position_command.lon_int = geoCoords.longitude.get() * (1e7);
    position_command.alt = geoCoords.altitude.get();
    position_command.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            POSITION_TARGET_TYPEMASK_FORCE_SET | POSITION_TARGET_TYPEMASK_YAW_IGNORE |   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    position_command.target_system = targetSystem;
    position_command.target_component = targetComponent;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_global_int_encode(targetSystem, targetComponent, &msg, &position_command);
    queueMessage(msg, TelemetryConditions::getCheckTargetGlobal(position_command.lat_int, position_command.lon_int, position_command.alt, targetSystem), 15, 3);

    simtime_t travelTime = distance / speed;

    cancelEvent(waypointChangeMessage);
    scheduleAt(simTime() + travelTime + waitTime, waypointChangeMessage);
}

void MAVLinkRandomWaypointMobility::startMovement() {
    EV_DEBUG << "Setting speed" << std::endl;
    // Sending MODE GUIDED command
    mavlink_command_long_t cmd;
    mavlink_message_t msg;
    cmd.command = MAV_CMD_DO_CHANGE_SPEED;
    cmd.confirmation = 0;
    cmd.param1 = 0;
    cmd.param2 = speed;
    cmd.target_component = 1;
    cmd.target_system = 1;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    queueMessage(msg, TelemetryConditions::getCheckPreArm(systemId), 15, 3);

    EV_DEBUG << "Sending GUIDED" << std::endl;
    cmd = {};
    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.confirmation = 0;
    cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    cmd.param2 = COPTER_MODE_GUIDED;
    cmd.target_component = 1;
    cmd.target_system = 1;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    queueMessage(msg, TelemetryConditions::getCheckPreArm(systemId), 15, 3);

    EV_DEBUG << "Sending ARM THROTTLE" << std::endl;
    cmd = {};
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 1;
    cmd.target_component = 1;
    cmd.target_system = 1;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    queueMessage(msg, TelemetryConditions::getCheckPreArm(systemId), 15, 3);


    EV_DEBUG << "Sending TAKEOFF" << std::endl;
    cmd = {};
    cmd.command = MAV_CMD_NAV_TAKEOFF;
    cmd.confirmation = 0;
    cmd.param7 = 25;
    cmd.target_component = 1;
    cmd.target_system = 1;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    queueMessage(msg, TelemetryConditions::getCheckPreArm(systemId), 15, 3);
}

}//namespace
