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

#ifndef __RT_EXPERIMENT_MAVLINKMANAGER_H_
#define __RT_EXPERIMENT_MAVLINKMANAGER_H_

#include <omnetpp.h>
#include "inet/common/scheduler/RealTimeScheduler.h"
#include "mavsimnet/utils/VehicleTypes.h"
#include "mavlink/ardupilotmega/mavlink.h"

using namespace omnetpp;
using namespace inet;

namespace mavsimnet {


typedef std::tuple<uint8_t, uint8_t> VehicleEntry;
class MAVLinkManager : public cSimpleModule, public RealTimeScheduler::ICallback
{
public:
    class IMAVLinkVehicle {
    public:
        virtual void receiveTelemetry(mavlink_message_t) = 0;
        VehicleType vehicleType;
    };

    virtual void startSimulator(VehicleType vehicleType, uint8_t systemId);
    virtual void registerVehicle(IMAVLinkVehicle *vehicle, uint8_t systemId, uint8_t componentId);
    virtual bool sendMessage(const mavlink_message_t& message, uint8_t destinationId);
    virtual bool notify(int fd) override;

    virtual uint8_t getSystemId() { return systemId; };
    virtual uint8_t getComponentId() { return componentId; };
    ~MAVLinkManager();

protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override;
    virtual void openSocket();

protected:
    uint8_t systemId, componentId;
    int fd, connectionPort;
    char buf[256];
    RealTimeScheduler *rtScheduler;
    std::map<uint8_t, struct sockaddr_in> addressMap;
    std::map<VehicleEntry, IMAVLinkVehicle*> registeredVehicles;
    std::string shellCommand;
    std::string simulatorPath;
    std::string MAVProxyPath;

private:

};

}

#endif
