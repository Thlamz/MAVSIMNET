/*
 * VehicleRoutines.cc
 *
 *  Created on: 12 de mar de 2022
 *      Author: thlam
 */

#include "VehicleRoutines.h"
#include "mavlink/ardupilotmega/mavlink.h"
#include "mavsimnet/utils/TelemetryConditions.h"

namespace mavsimnet {
namespace VehicleRoutines {

std::vector<Instruction> armTakeoffCopter(float altitude, uint8_t targetSystem, uint8_t targetComponent) {
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
    cmd.param7 = altitude;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    instructions.push_back({msg, TelemetryConditions::getCheckAltitude(25, 3, targetSystem), 30, 3});

    return instructions;
}

std::vector<Instruction> armTakeoffPlane(float altitude, uint8_t targetSystem, uint8_t targetComponent) {
    std::vector<Instruction> instructions;

    mavlink_command_long_t cmd;
    mavlink_message_t msg;

    mavlink_param_set_t set_altitude {
        altitude,
        targetSystem,
        targetComponent,
        "TKOFF_ALT",
        MAV_PARAM_TYPE_REAL32
    };
    mavlink_msg_param_set_encode(targetSystem, targetComponent, &msg, &set_altitude);
    instructions.push_back({msg, TelemetryConditions::getCheckParamValue("TKOFF_ALT", altitude, targetSystem), 30, 3});

    for(Instruction instruction : setMode(PLANE, TAKEOFF, targetSystem, targetComponent)) {
        instructions.push_back(instruction);
    }

    cmd = {};
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = 1;
    cmd.target_component = 1;
    cmd.target_system = 1;

    mavlink_msg_command_long_encode(targetSystem, targetComponent, &msg, &cmd);
    instructions.push_back({msg, TelemetryConditions::getCheckAltitude(altitude, 3, targetSystem), 15, 3});

    return instructions;
}

std::vector<Instruction> armTakeoff(VehicleType type, float altitude, uint8_t targetSystem, uint8_t targetComponent) {
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

// https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
std::vector<Instruction> guidedGotoCopter(double latitude, double longitude, float altitude, inet::IGeographicCoordinateSystem *coordinateSystem, uint8_t targetSystem, uint8_t targetComponent) {
    std::vector<Instruction> instructions;

    mavlink_set_position_target_global_int_t position_command;
    position_command.lat_int = latitude * (1e7);
    position_command.lon_int = longitude * (1e7);
    position_command.alt = altitude;
    position_command.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
           POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
           POSITION_TARGET_TYPEMASK_YAW_IGNORE |   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    position_command.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    position_command.target_system = targetSystem;
    position_command.target_component = targetComponent;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_global_int_encode(targetSystem, targetComponent, &msg, &position_command);
    instructions.push_back({msg, TelemetryConditions::getCheckGlobalPosition(latitude, longitude, altitude, 5, coordinateSystem, targetSystem), 15, 3});

    return instructions;
}

// https://ardupilot.org/dev/docs/plane-commands-in-guided-mode.html
std::vector<Instruction> guidedGotoPlane(double latitude, double longitude, float altitude, uint8_t targetSystem, uint8_t targetComponent) {
    std::vector<Instruction> instructions;

    mavlink_mission_item_int_t missionItem;
    missionItem.target_system = targetSystem;
    missionItem.target_component = targetComponent;
    missionItem.x = latitude * (1e7);
    missionItem.y = longitude * (1e7);
    missionItem.z = altitude;
    missionItem.command = MAV_CMD_NAV_WAYPOINT;
    missionItem.frame = MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
    missionItem.current = 2; // Current 2 to indicate goto command

    mavlink_message_t msg;
    mavlink_msg_mission_item_int_encode(targetSystem, targetComponent, &msg, &missionItem);
    instructions.push_back({msg, TelemetryConditions::getCheckMissionItemReached(0, targetSystem), 15, 3});

    return instructions;
}

std::vector<Instruction> guidedGoto(VehicleType type, double latitude, double longitude, float altitude, inet::IGeographicCoordinateSystem *coordinateSystem, uint8_t targetSystem, uint8_t targetComponent) {
    switch(type) {
    case COPTER:
        return guidedGotoCopter(latitude, longitude, altitude, coordinateSystem, targetSystem, targetComponent);
    case PLANE:
        return guidedGotoPlane(latitude, longitude, altitude, targetSystem, targetComponent);
    }
}


}
}