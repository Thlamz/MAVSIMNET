#include <iostream>

#include "TelemetryConditions.h"
/**
 * @brief This module contains message checkers that should be used as conditions for requirements.
 * 
 */


bool verifySender(mavlink_message_t message, uint8_t senderSystemId) {
    return (message.sysid == senderSystemId);
}


/**
 * @brief An empty checker, always immediatly resolves
 * 
 * @return true 
 */
bool TelemetryConditions::checkEmpty(mavlink_message_t) {
    return true;
}


/**
 * @brief Gets the check_cmd_ack message checker for a specific command
 * 
 * @param command The command you want to get the checker for
 * @return Condition Returns true when command gets acknowledged
 */
std::function<bool(mavlink_message_t)> TelemetryConditions::getCheckCmdAck(uint8_t systemId, uint8_t componentId, unsigned short command, uint8_t senderSystemId) {
    return [=](mavlink_message_t message) {
        if(message.msgid == MAVLINK_MSG_ID_COMMAND_ACK && verifySender(message, senderSystemId)) {
            mavlink_command_ack_t ack;

            mavlink_msg_command_ack_decode(&message, &ack);
            return (ack.command == command && ack.result == MAV_RESULT_ACCEPTED && ack.target_system == systemId && ack.target_component == componentId);
        }
        return false;
    };
}

/**
 * @brief Checks if the vehicle is ready to arm
 * 
 * @param message Telemetry recieved
 * @return true If the vehicle is ready to arm
 * @return false If the vehicle is not yet ready to arm 
 */
std::function<bool(mavlink_message_t)> TelemetryConditions::getCheckPreArm(uint8_t senderSystemId) {
    static const uint16_t required_value = (EKF_ATTITUDE |
                ESTIMATOR_VELOCITY_HORIZ |
                ESTIMATOR_VELOCITY_VERT |
                ESTIMATOR_POS_HORIZ_REL |
                ESTIMATOR_PRED_POS_HORIZ_REL);
                    
    static const uint16_t error_bits = (ESTIMATOR_CONST_POS_MODE |
                ESTIMATOR_ACCEL_ERROR);
    return [=](mavlink_message_t message) {
        if(message.msgid == MAVLINK_MSG_ID_EKF_STATUS_REPORT && verifySender(message, senderSystemId)) {
            mavlink_ekf_status_report_t report;
            mavlink_msg_ekf_status_report_decode(&message, &report);

            // Flags don't include error bits and include all required values
            return !(report.flags & error_bits) && ((report.flags & required_value) == required_value);
        }
        return false;
    };
}

/**
 * @brief Checks if the vehicle is armed
 * 
 * @param message Telemetry received
 * @return true If the vehicle is armed
 * @return false If the vehicle is not armed
 */
std::function<bool(mavlink_message_t)> TelemetryConditions::getCheckArm(uint8_t senderSystemId) {
    return [=](mavlink_message_t message) {
        if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT && verifySender(message, senderSystemId)) {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            return static_cast<bool>(heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
        }
        return false;
    };
}

/**
 * @brief Gets a message checker that will verify if a vehicle is within tolerance of an altutude
 * 
 * @param altitude Target altitude
 * @param tolerance Margin of error for the altitude
 * @return Condition Message checker for that specific altitude and tolerance
 */
std::function<bool(mavlink_message_t)> TelemetryConditions::getCheckAltitude(int32_t altitude, int32_t tolerance, uint8_t senderSystemId) {
    return [=](mavlink_message_t msg) {
        if(msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && verifySender(msg, senderSystemId)) {
            mavlink_global_position_int_t position;
            mavlink_msg_global_position_int_decode(&msg, &position);
            return abs(position.relative_alt / 1000 - altitude) < tolerance;
        }
        return false;
    };
}

/**
 * @brief Gets a message checker that will resolve when a mission request arrives
 *
 */
std::function<bool(mavlink_message_t)> TelemetryConditions::getCheckMissionRequest(uint8_t systemId, uint8_t componentId, uint16_t sequenceNumber, uint8_t senderSystemId) {
    return [=](mavlink_message_t msg) {
        if(msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT && verifySender(msg, senderSystemId)) {
            mavlink_mission_request_int_t request;
            mavlink_msg_mission_request_int_decode(&msg, &request);
            return (request.seq == sequenceNumber) && (request.target_system == systemId) && (request.target_component == componentId);
        }
        else if(msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST && verifySender(msg, senderSystemId)) {
            mavlink_mission_request_t request;
            mavlink_msg_mission_request_decode(&msg, &request);
            return (request.seq == sequenceNumber) && (request.target_system == systemId) && (request.target_component == componentId);
        }
        return false;
    };
}

std::function<bool(mavlink_message_t)> TelemetryConditions::getCheckMissionAck(uint8_t systemId, uint8_t componentId, uint8_t senderSystemId) {
    return [=](mavlink_message_t msg) {
        if(msg.msgid == MAVLINK_MSG_ID_MISSION_ACK && verifySender(msg, senderSystemId)) {
                mavlink_mission_ack_t ack;
                mavlink_msg_mission_ack_decode(&msg, &ack);
                return (ack.type == MAV_MISSION_ACCEPTED && ack.target_system == systemId && ack.target_component == componentId);
        }
        return false;
    };
}

std::function<bool(mavlink_message_t)> TelemetryConditions::getCheckTargetGlobal(float lat, float lon, float alt, uint8_t senderSystemId) {
    return [=](mavlink_message_t msg) {
        if(msg.msgid == MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT && verifySender(msg, senderSystemId)) {
            mavlink_position_target_global_int_t position;
            mavlink_msg_position_target_global_int_decode(&msg, &position);

            return (position.lat_int == lat) && (position.lon_int == lon) && (position.alt = alt);
        }
        return false;
    };
}
