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

#include "MAVLinkMobilityBase.h"
#include "mavsimnet/utils/TelemetryConditions.h"

using namespace omnetpp;
using namespace inet;

namespace mavsimnet {


Define_Module(MAVLinkMobilityBase);

void MAVLinkMobilityBase::initialize(int stage)
{
    MovingMobilityBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        targetSystem = (par("targetSystem").intValue() == -1) ? getId() : par("targetSystem");
        targetComponent = par("targetComponent");
        vehicleType = static_cast<VehicleType>(par("vehicleType").intValue());
        manager = getModuleFromPar<MAVLinkManager>(par("managerModule"), this, true);
        coordinateSystem = getModuleFromPar<IGeographicCoordinateSystem>(par("coordinateSystemModule"), this, true);
        manager->registerVehicle(this, targetSystem, targetComponent, par("paramPath"));
    }
    if (stage == 1) {
        systemId = manager->getSystemId();
        componentId = manager->getComponentId();
        performInitialSetup();
    }
}
void MAVLinkMobilityBase::handleMessage(cMessage *msg) {
    Enter_Method_Silent();
    if(msg->isSelfMessage()) {
        if(strcmp(msg->getName(), "MAVLinkMobilityBaseMessage") == 0) {
            switch(msg->getKind()) {
                case CommunicationSelfMessages::TIMEOUT:
                    if(activeInstructionTries < getActiveRetries()) {
                        EV_WARN << "Timeout reached" << std::endl;
                        activeInstructionTries++;
                        sendActiveMessage();

                        // Setting up timeout again if there are any retries left
                        if(getActiveTimeout() > 0) {
                            scheduleAt(simTime() + getActiveTimeout(), timeoutMessage);
                        }
                    } else {
                        EV_WARN << "Max retries reached." << activeInstructionTries << std::endl;
                        nextMessage();
                    }
                    return;
                default:
                    return;
            }
        }
    }
    MovingMobilityBase::handleMessage(msg);
}



void MAVLinkMobilityBase::performInitialSetup() {
    mavlink_command_long_t cmd;
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.confirmation = 0;
    cmd.param1 = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    cmd.param2 = updateInterval.inUnit(SimTimeUnit::SIMTIME_US);
    cmd.param7 = 0;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(systemId, componentId, &message, &cmd);
    queueMessage(message,
            TelemetryConditions::getCheckCmdAck(systemId, componentId, MAV_CMD_SET_MESSAGE_INTERVAL, targetSystem),
            15, 5, "Setting stream rate for position messages");

    cmd = {};
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.confirmation = 0;
    cmd.param1 = MAVLINK_MSG_ID_ATTITUDE;
    cmd.param2 =  updateInterval.inUnit(SimTimeUnit::SIMTIME_US);
    cmd.param7 = 0;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;
    mavlink_msg_command_long_encode(systemId, componentId, &message, &cmd);
    queueMessage(message,
            TelemetryConditions::getCheckCmdAck(systemId, componentId, MAV_CMD_SET_MESSAGE_INTERVAL, targetSystem),
            15, 5, "Setting stream rate for attitude messages");

    cmd = {};
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.confirmation = 0;
    cmd.param1 = MAVLINK_MSG_ID_EKF_STATUS_REPORT;
    cmd.param2 = 2000000; // 0.5 Hz
    cmd.param7 = 0;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;
    mavlink_msg_command_long_encode(systemId, componentId, &message, &cmd);
    queueMessage(message,
            TelemetryConditions::getCheckCmdAck(systemId, componentId, MAV_CMD_SET_MESSAGE_INTERVAL, targetSystem),
            15, 5, "Setting stream rate for ekf reports");

    cmd = {};
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.confirmation = 0;
    cmd.param1 = MAVLINK_MSG_ID_HEARTBEAT;
    cmd.param2 = 2000000; // 0.5 Hz
    cmd.param7 = 0;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;
    mavlink_msg_command_long_encode(systemId, componentId, &message, &cmd);
    queueMessage(message,
            TelemetryConditions::getCheckCmdAck(systemId, componentId, MAV_CMD_SET_MESSAGE_INTERVAL, targetSystem),
            15, 5, "Setting stream rate for heart beats");

    cmd = {};
    cmd.command = MAV_CMD_DO_SET_HOME;
    cmd.confirmation = 0;
    cmd.param1 = 1; // Set current location as home
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;
    mavlink_msg_command_long_encode(systemId, componentId, &message, &cmd);
    queueMessage(message,
            TelemetryConditions::getCheckCmdAck(systemId, componentId, MAV_CMD_DO_SET_HOME, targetSystem),
            15, 5, "Setting home");
}

void MAVLinkMobilityBase::queueMessage(mavlink_message_t message, Condition condition, simtime_t timeout, int retries, std::string label) {
    instructionQueue.push(std::make_shared<Instruction>(message, condition, timeout, retries, label));
}

void MAVLinkMobilityBase::queueInstruction(std::shared_ptr<Instruction> instruction) {
    instructionQueue.push(instruction);
}

void MAVLinkMobilityBase::queueInstructions(std::vector<std::shared_ptr<Instruction>> instructions) {
    for (std::shared_ptr<Instruction> instruction : instructions) {
        instructionQueue.push(instruction);
    }
}

void MAVLinkMobilityBase::nextMessage() {
    EV_DETAIL << "Proceeding to next message" << std::endl;
    cancelEvent(timeoutMessage);

    if(instructionQueue.size() > 0) {
        activeInstructionTries = 0;
        activeInstruction = instructionQueue.front();
        instructionQueue.pop();

        if(!getActiveLabel().empty()) {
            EV_INFO << "Setting \"" << getActiveLabel() << "\" instruction to active." << std::endl;
        }

        sendActiveMessage();


        // Setting up timeout
        if(getActiveTimeout() > 0) {
            scheduleAt(simTime() + getActiveTimeout(), timeoutMessage);
        }
    }
}

void MAVLinkMobilityBase::nextMessageIfReady() {
    if(!getActiveCondition() || getActiveCompleted()) {
        nextMessage();
    }
}

void MAVLinkMobilityBase::clearQueue() {
    EV_DETAIL << "Clearing message queue" << std::endl;

    activeInstruction = nullptr;
    instructionQueue = {};
    activeInstructionTries = 0;
}

bool MAVLinkMobilityBase::sendActiveMessage() {
    if(activeInstruction != nullptr) {
        if(getActiveLabel().empty()) {
            EV_INFO << "(" << +targetSystem << ") Sending message: " << getActiveMessage().msgid << std::endl;
        }
        else {
            EV_INFO << "(" << +targetSystem << ") Sending message: " << getActiveLabel() << " - " << getActiveMessage().msgid << std::endl;
        }
        bool success = sendMessage(getActiveMessage(), getActiveTimeout() == 0,activeInstructionTries, getActiveRetries());
        if(!success && getActiveTimeout() == 0) {
            nextMessage();
            return false;
        }
        return success;
    }
    return false;
}

bool MAVLinkMobilityBase::sendMessage(const mavlink_message_t& message, bool shouldRetry, int &currentTries, int maxRetries) {
    do {
        if(currentTries >= 1) {
            EV_WARN << "RETRY " << currentTries << std::endl;
        }

        if(!manager->sendMessage(message, targetSystem)) {
            EV_WARN << "Failed to send message" << std::endl;
            if (shouldRetry == true) {
                currentTries++;
                continue;
            } else {
                return false;
            }
            continue;
        } else {
            EV_INFO << "Message sent: " << message.msgid << std::endl;
            return true;
        }
    } while(currentTries < maxRetries);
    EV_WARN << "Max retries reached: " << activeInstructionTries << std::endl;
    return false;
}

void MAVLinkMobilityBase::move() {
    lastVelocity = (currentPosition - lastPosition) * (1/(simTime() - lastUpdate));
    lastPosition = currentPosition;
}

void MAVLinkMobilityBase::orient() {
    lastOrientation = currentOrientation;
}


void MAVLinkMobilityBase::updatePosition(const mavlink_message_t& msg) {
    // Updating vehicle position
    if(msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t position;
        mavlink_msg_global_position_int_decode(&msg, &position);

        currentPosition = coordinateSystem->computeSceneCoordinate(GeoCoord(deg(((double)position.lat)/1e7),
                deg(((double)position.lon)/1e7),
                mm(((double)position.relative_alt))));
    }

    // Update vehicle orientation
    if(msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);

        currentOrientation = Quaternion(EulerAngles(rad(M_PI - attitude.yaw), rad(-attitude.roll), rad(-attitude.pitch)));
    }
}


void MAVLinkMobilityBase::receiveTelemetry(mavlink_message_t message) {
    Enter_Method_Silent();

    EV_DETAIL << "(" << +targetSystem << ") Received MAVLINK: " << message.msgid << std::endl;
    updatePosition(message);

    if(getActiveCondition() && getActiveCondition()(message) && activeInstruction != nullptr && !getActiveCompleted()) {
        activeInstruction->completed = true;
        EV_INFO << "Message " << getActiveMessage().msgid << " done." << std::endl;
    }

    nextMessageIfReady();
}

void MAVLinkMobilityBase::finish() {
    MovingMobilityBase::finish();
    cancelAndDelete(timeoutMessage);
    clearQueue();
}

}
