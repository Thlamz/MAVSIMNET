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

#include "MAVLinkManager.h"
#include <winsock2.h>
#include <fstream>

namespace mavsimnet {

Define_Module(MAVLinkManager);

void MAVLinkManager::initialize(int stage) {
    if (stage == INITSTAGE_LOCAL) {
        systemId = par("systemId");
        componentId = par("componentId");

        rtScheduler = check_and_cast<inet::RealTimeScheduler *>(getSimulation()->getScheduler());
        basePort = par("basePort");

        copterSimulatorPath = par("copterSimulatorPath").stdstringValue();
        planeSimulatorPath = par("planeSimulatorPath").stdstringValue();
        roverSimulatorPath = par("roverSimulatorPath").stdstringValue();

    }
}

int MAVLinkManager::numInitStages() const {
    return 3;
}

void MAVLinkManager::startSimulator(VehicleType vehicleType, uint8_t systemId, std::string paramPath) {
    std::string command;

    switch(vehicleType) {
    case COPTER:
        command += copterSimulatorPath;
        command += " -M quad -w --defaults " + paramPath;
        break;
    case PLANE:
        command += planeSimulatorPath;
        command += " -M plane -w --defaults " + paramPath;
        break;
    case ROVER:
        command += roverSimulatorPath;
        command += " -M rover -w --defaults " + paramPath;
    }

    command += " --base-port ";
    command += std::to_string(basePort + (systemId * 10));
    command += " --sysid ";
    command += std::to_string(+systemId);

    EV_INFO << "Starting simulator with command: " << command << std::endl;

    TinyProcessLib::Process *process = new TinyProcessLib::Process(command, ".");
    simulatorProcesses.push_back(process);
}

// TODO: Support for real vehicles, specifying address and port
void MAVLinkManager::registerVehicle(IMAVLinkVehicle *vehicle, uint8_t systemId, uint8_t componentId, std::string paramPath) {
    EV_INFO << "Registering vehicle with sysid: " << +systemId << " and componentid: " << +componentId << std::endl;
    VehicleEntry entry  { systemId, componentId };
    registeredVehicles.insert({entry, vehicle});

    startSimulator(vehicle->vehicleType, systemId, paramPath);
    openSocket(entry, basePort + (systemId * 10));
}

bool MAVLinkManager::sendMessage(const mavlink_message_t& message, uint8_t destinationId) {
    Enter_Method_Silent();
    EV_DETAIL << "Sending message to: " << +destinationId << std::endl;
    int length = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);
    static const int c = sizeof(sockaddr_in);

    int fd = -1;
    for(auto& pair : sockets) {
        if(pair.first.first == destinationId) {
            fd = pair.second;
            break;
        }
    }

    if(fd != -1) {
        if(::send(fd, buf, length, 0) == SOCKET_ERROR) {
            EV_ERROR << "Error sending message: " << WSAGetLastError() << std::endl;
            return false;
        }
        return true;
    }
    return false;
}


void MAVLinkManager::openSocket(VehicleEntry vehicle, int port)
{
    WSADATA wsa;
    struct sockaddr_in server;

    EV_INFO << "Initialising Winsock..." << std::endl;
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        EV_ERROR << "Failed. Error Code : %d\n",WSAGetLastError();
        return;
    }

    EV_INFO << "Initialised" << std::endl;

    //Create a socket
    // TODO: This should be a UDP socket if we are connecting to a real vehicle
    int fd;
    if((fd = socket(AF_INET , SOCK_STREAM , 0)) == INVALID_SOCKET)
    {
        EV_ERROR << "Could not create socket : " << WSAGetLastError() << std::endl;
        return;
    }

    sockets.push_back({vehicle, fd});

    EV_INFO << "Socket created" << std::endl;

    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    server.sin_port = htons( port );

    //Bind
    if( connect(fd ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
    {
        EV_ERROR << "Connect failed with error code : " << WSAGetLastError() << std::endl;
        return;
    }
    EV_INFO << "Socket bound" << std::endl;


    u_long mode = 1;  // 1 to enable non-blocking socket
    if (ioctlsocket(fd, FIONBIO, &mode) == SOCKET_ERROR) {
        EV_ERROR << "Setting socket to non-blocking failed with code : " << WSAGetLastError() << std::endl;
        return;
    }
    EV_INFO << "Socket set to non-blocking" << std::endl;

    rtScheduler->addCallback(fd, this);
}


bool MAVLinkManager::notify(int incoming) {
    Enter_Method_Silent();
    EV_DETAIL << "Notified" << std::endl;

    VehicleEntry vehicleEntry;
    bool found = false;
    for (auto& pair : sockets) {
        if(pair.second == incoming) {
            vehicleEntry = pair.first;
            found = true;
            break;
        }
    }

    if(found) {
        int length;

        do {
            if((length = recv(incoming, buf, 256, 0)) == SOCKET_ERROR) {
                int error = WSAGetLastError();
                EV_DEBUG << "Received WSAEWOULDBLOCK, the socket is empty." << std::endl;
                // Blocking errors are normal when no messages are present in a non-blocking socket
                if(error == WSAEWOULDBLOCK) return true;
                EV_ERROR << "Error receiving message, code: " << error << std::endl;
                return false;
            }
            EV_DETAIL << "Received " << length << " bytes." << std::endl;

            mavlink_status_t status;
            mavlink_message_t msg;

            for (int i = 0; i < length; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
                {
                    MAVLinkManager::IMAVLinkVehicle* vehicle = registeredVehicles[vehicleEntry];
                    vehicle->receiveTelemetry(msg);
                }
            }
        } while(length > 0);
        return true;
    }
    return false;
}


void MAVLinkManager::finish() {
    WSACleanup();
    for (auto &pair : sockets) {
        int fd = pair.second;
        rtScheduler->removeCallback(fd, this);
        closesocket(fd);
    }

    for(TinyProcessLib::Process *subprocess : simulatorProcesses) {
        if(subprocess != nullptr) {
            subprocess->kill();
        }
    }
}


}

