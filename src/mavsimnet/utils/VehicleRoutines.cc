/*
 * VehicleRoutines.cc
 *
 *  Created on: 12 de mar de 2022
 *      Author: thlam
 */

#include <omnetpp.h>
#include "VehicleRoutines.h"
#include "mavlink/ardupilotmega/mavlink.h"
#include "mavsimnet/utils/TelemetryConditions.h"

namespace mavsimnet {
namespace VehicleRoutines {

template<>
std::vector<Instruction> armTakeoff<VehicleType::COPTER>(double altitude, uint8_t targetSystem, uint8_t targetComponent) {
    std::vector<Instruction> instructions(3);

    mavlink_command_long_t cmd;
    mavlink_message_t msg;

    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.confirmation = 0;
    cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    cmd.param2 = COPTER_MODE_GUIDED;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    instructions.push_back({msg, TelemetryConditions::getCheckPreArm(targetSystem), 30, 3});

    cmd = {};
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 1;
    cmd.target_component = 1;
    cmd.target_system = 1;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    instructions.push_back({msg, TelemetryConditions::getCheckArm(targetSystem), 15, 3});

    cmd = {};
    cmd.command = MAV_CMD_NAV_TAKEOFF;
    cmd.confirmation = 0;
    cmd.param7 = 25;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    instructions.push_back({msg, TelemetryConditions::getCheckAltitude(25, 3, targetSystem), 30, 3});

    return instructions;
}

}
}
