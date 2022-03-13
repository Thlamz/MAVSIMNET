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

#ifndef __RT_EXPERIMENT_COMMUNICATIONTESTMODULE_H_
#define __RT_EXPERIMENT_COMMUNICATIONTESTMODULE_H_

#include <omnetpp.h>
#include <queue>
#include <functional>
#include "inet/mobility/base/MovingMobilityBase.h"
#include "inet/common/geometry/common/GeographicCoordinateSystem.h"
#include "mavlink/ardupilotmega/mavlink.h"
#include "mavsimnet/manager/MAVLinkManager.h"
#include "mavsimnet/mobility/base/MAVLinkInstruction.h"
#include "mavsimnet/utils/VehicleTypes.h"

namespace mavsimnet {

class MAVLinkMobilityBase : public MovingMobilityBase, public MAVLinkManager::IMAVLinkVehicle
{
  public:
    virtual void receiveTelemetry(mavlink_message_t message) override;

  protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void move() override;
    virtual void orient() override;

    virtual void establishConnection();
    virtual void sendMessage(Instruction instruction);
    virtual void updatePosition(mavlink_message_t message);
    virtual void queueMessage(mavlink_message_t message, Condition condition = {}, simtime_t timeout = -1, int retries = 0);
    virtual void queueInstruction(Instruction instruction);
    virtual void queueInstructions(std::vector<Instruction> instructions);
    virtual void clearQueue();

    virtual void nextMessage();
    virtual void sendMessage(mavlink_message_t message, int &currentTries, int maxRetries);

    mavlink_message_t getActiveMessage() { return activeInstruction.message; };
    Condition getActiveCondition() { return activeInstruction.condition; };
    simtime_t getActiveTimeout() { return activeInstruction.timeout; };
    int getActiveRetries() { return activeInstruction.retries; };
    Coord getCurrentCoord() { return currentPosition; }

    ~MAVLinkMobilityBase();


    enum CommunicationSelfMessages {
        TIMEOUT = 0
    };

  protected:
    uint8_t systemId, componentId, targetSystem, targetComponent;
    Coord currentPosition;
    Quaternion currentOrientation;
    MAVLinkManager *manager;
    IGeographicCoordinateSystem *coordinateSystem;
  private:
    int fd;
    std::queue<Instruction> instructionQueue;

    Instruction activeInstruction;
    int activeInstructionTries = 0;

    cMessage *timeoutMessage = new cMessage("MAVLinkMobilityBaseMessage", CommunicationSelfMessages::TIMEOUT);
};


}
#endif
