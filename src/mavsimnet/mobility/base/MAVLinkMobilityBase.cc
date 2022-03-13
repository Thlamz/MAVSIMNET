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
        targetSystem = par("targetSystem");
        targetComponent = par("targetComponent");
        manager = getModuleFromPar<MAVLinkManager>(par("managerModule"), this, true);
        coordinateSystem = getModuleFromPar<IGeographicCoordinateSystem>(par("coordinateSystemModule"), this, true);
        manager->registerVehicle(this, targetSystem, targetComponent);
        establishConnection();
    }

    if (stage == 1) {
        systemId = manager->getSystemId();
        componentId = manager->getComponentId();

    }
}
void MAVLinkMobilityBase::handleMessage(cMessage *msg) {
    if(msg->isSelfMessage()) {
        if(strcmp(msg->getName(), "MAVLinkMobilityBaseMessage") == 0) {
            switch(msg->getKind()) {
                case CommunicationSelfMessages::TIMEOUT:
                    if(activeInstructionTries < getActiveRetries()) {
                        EV_WARN << "Timeout reached" << std::endl;
                        activeInstructionTries++;
                        sendMessage(activeInstruction);

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



void MAVLinkMobilityBase::establishConnection() {
    EV_INFO << "Setting STREAM RATE FOR POSITION" << std::endl;
    mavlink_command_long_t cmd;
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.confirmation = 0;
    cmd.param1 = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    cmd.param2 = 100000; // 10 Hz
    cmd.param7 = 0;
    cmd.target_component = targetComponent;
    cmd.target_system = targetSystem;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(targetSystem, targetComponent, &message, &cmd);
    queueMessage(message, TelemetryConditions::checkEmpty, 15, 5);
}

void MAVLinkMobilityBase::queueMessage(mavlink_message_t message, Condition condition, simtime_t timeout, int retries) {
    instructionQueue.push({message, condition, timeout, retries});
}

void MAVLinkMobilityBase::queueInstruction(Instruction instruction) {
    instructionQueue.push(instruction);
}

void MAVLinkMobilityBase::nextMessage() {
    EV_DETAIL << "Proceeding to next message" << std::endl;
    cancelEvent(timeoutMessage);

    if(instructionQueue.size() > 0) {
        activeInstructionTries = 0;
        activeInstruction = instructionQueue.front();
        instructionQueue.pop();

        sendMessage(activeInstruction);

        // Setting up timeout
        if(getActiveTimeout() > 0) {
            scheduleAt(simTime() + getActiveTimeout(), timeoutMessage);
        }
    }
}

void MAVLinkMobilityBase::clearQueue() {
    EV_DETAIL << "Clearing message queue" << std::endl;

    activeInstruction = {};
    instructionQueue = {};
    activeInstructionTries = 0;
}

void MAVLinkMobilityBase::sendMessage(Instruction instruction) {
    sendMessage(getActiveMessage(), activeInstructionTries, getActiveRetries());
}

void MAVLinkMobilityBase::sendMessage(mavlink_message_t message, int &currentTries, int maxRetries) {
    EV_INFO << "Sending message: " << message.msgid << std::endl;
    do {
        if(currentTries >= 1) {
            EV_WARN << "RETRY " << currentTries << std::endl;
        }

        if(!manager->sendMessage(message, targetSystem)) {
            EV_WARN << "Failed to send message" << std::endl;
            if (getActiveTimeout() == 0) {
                currentTries++;
                continue;
            } else {
                return;
            }
            continue;
        } else {
            EV_INFO << "Message sent: " << message.msgid << std::endl;
            return;
        }
    } while(currentTries < maxRetries);
    EV_WARN << "Max retries reached: " << activeInstructionTries << std::endl;
    nextMessage();
}

void MAVLinkMobilityBase::move() {
    lastVelocity = (currentPosition - lastPosition) * (1/(simTime() - lastUpdate));
    lastPosition = currentPosition;
}

void MAVLinkMobilityBase::orient() {
    lastOrientation = currentOrientation;
}


void MAVLinkMobilityBase::updatePosition(mavlink_message_t msg) {
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

        currentOrientation =  Quaternion(EulerAngles(rad(M_PI - attitude.yaw), rad(-attitude.roll), rad(-attitude.pitch)));
    }
}


void MAVLinkMobilityBase::receiveTelemetry(mavlink_message_t message) {
    Enter_Method_Silent();

    EV_DETAIL << "Received MAVLINK: " << message.msgid << std::endl;
    updatePosition(message);
    if(!getActiveCondition() || getActiveCondition()(message)) {
        nextMessage();
    }
}

MAVLinkMobilityBase::~MAVLinkMobilityBase() {
    cancelAndDelete(timeoutMessage);
}

}
