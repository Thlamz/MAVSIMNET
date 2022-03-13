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

std::vector<Instruction> armTakeoffCopter(int altitude, uint8_t targetSystem, uint8_t targetComponent) {
    std::vector<Instruction> instructions;

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

std::vector<Instruction> armTakeoffPlane(int altitude, uint8_t targetSystem, uint8_t targetComponent) {
    std::vector<Instruction> instructions;

    mavlink_command_long_t cmd;
    mavlink_message_t msg;

    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.confirmation = 0;
    cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    cmd.param2 = PLANE_MODE_TAKEOFF;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    instructions.push_back({msg, TelemetryConditions::getCheckPreArm(targetSystem), 30, 3});

//    mavlink_param_set_t set_altitude {
//        altitude,
//        targetSystem,
//        targetComponent,
//        "TKOFF_ALT",
//        MAV_PARAM_TYPE_INT16
//    };
//    mavlink_msg_param_set_encode(targetSystem, targetComponent, &msg, &set_altitude);
//    instructions.push_back({msg, TelemetryConditions::getCheckParamValue("TKOFF_ALT", MAV_PARAM_TYPE_INT16, altitude, targetSystem), 30, 3});

    cmd = {};
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 1;
    cmd.target_component = 1;
    cmd.target_system = 1;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    instructions.push_back({msg, TelemetryConditions::getCheckArm(targetSystem), 15, 3});

    return instructions;
}

std::vector<Instruction> armTakeoff(VehicleType type, int altitude, uint8_t targetSystem, uint8_t targetComponent) {
    switch(type) {
    case COPTER:
        return armTakeoffCopter(altitude, targetSystem, targetComponent);
    case PLANE:
        return armTakeoffPlane(altitude, targetSystem, targetComponent);
    }
}

std::vector<Instruction> setModeCopter(Mode mode, uint8_t targetSystem, uint8_t targetComponent) {
    std::vector<Instruction> instructions;
    mavlink_command_long_t cmd;
    mavlink_message_t msg;
    switch(mode) {
    case GUIDED:
        cmd.command = MAV_CMD_DO_SET_MODE;
        cmd.confirmation = 0;
        cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        cmd.param2 = COPTER_MODE_GUIDED;
        cmd.target_component = targetComponent;
        cmd.target_system = targetSystem;

        mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
        instructions.push_back({msg, TelemetryConditions::getCheckCmdAck(targetSystem, targetComponent, MAV_CMD_DO_SET_MODE, targetSystem), 30, 3});
        break;
    case AUTO:
        cmd.command = MAV_CMD_DO_SET_MODE;
        cmd.confirmation = 0;
        cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        cmd.param2 = COPTER_MODE_AUTO;
        cmd.target_component = targetComponent;
        cmd.target_system = targetSystem;
        mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
        instructions.push_back({msg, TelemetryConditions::getCheckCmdAck(targetSystem, targetComponent, MAV_CMD_DO_SET_MODE, targetSystem), 15, 3});
        break;
    case TAKEOFF:
        break;
    }
    return instructions;
}

std::vector<Instruction> setModePlane(Mode mode, uint8_t targetSystem, uint8_t targetComponent) {
    std::vector<Instruction> instructions;
    mavlink_command_long_t cmd;
    mavlink_message_t msg;
    switch(mode) {
    case GUIDED:
        cmd.command = MAV_CMD_DO_SET_MODE;
        cmd.confirmation = 0;
        cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        cmd.param2 = PLANE_MODE_GUIDED;
        cmd.target_component = targetComponent;
        cmd.target_system = targetSystem;

        mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
        instructions.push_back({msg, TelemetryConditions::getCheckCmdAck(targetSystem, targetComponent, MAV_CMD_DO_SET_MODE, targetSystem), 30, 3});
        break;
    case AUTO:
        cmd.command = MAV_CMD_DO_SET_MODE;
        cmd.confirmation = 0;
        cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        cmd.param2 = PLANE_MODE_AUTO;
        cmd.target_component = targetComponent;
        cmd.target_system = targetSystem;
        mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
        instructions.push_back({msg, TelemetryConditions::getCheckCmdAck(targetSystem, targetComponent, MAV_CMD_DO_SET_MODE, targetSystem), 15, 3});
        break;
    case TAKEOFF:
        cmd.command = MAV_CMD_DO_SET_MODE;
        cmd.confirmation = 0;
        cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        cmd.param2 = PLANE_MODE_TAKEOFF;
        cmd.target_component = targetComponent;
        cmd.target_system = targetSystem;
        mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
        instructions.push_back({msg, TelemetryConditions::getCheckCmdAck(targetSystem, targetComponent, MAV_CMD_DO_SET_MODE, targetSystem), 15, 3});
        break;
    }
    return instructions;
}

std::vector<Instruction> setMode(VehicleType type, Mode mode, uint8_t targetSystem, uint8_t targetComponent) {
    switch(type) {
    case COPTER:
        return setModeCopter(mode, targetSystem, targetComponent);
    case PLANE:
        return setModePlane(mode, targetSystem, targetComponent);
    }
}


}
}
