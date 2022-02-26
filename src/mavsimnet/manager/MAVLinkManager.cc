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

namespace mavsimnet {

Define_Module(MAVLinkManager);

void MAVLinkManager::initialize(int stage) {
    if (stage == INITSTAGE_LOCAL) {
        rtScheduler = check_and_cast<inet::RealTimeScheduler *>(getSimulation()->getScheduler());
        connectionPort = par("connectionPort");

        shellCommand = par("shellCommand").stdstringValue();
        simulatorPath = par("simulatorPath").stdstringValue();
        MAVProxyPath = par("MAVProxyPath").stdstringValue();
    }
    if (stage == 1) {
        openSocket();
    }
}

int MAVLinkManager::numInitStages() const {
    return 2;
}

void MAVLinkManager::startSimulator(uint8_t systemId) {
    std::ostringstream command;
#ifdef _WIN32
    command << "set PATH=%PATH%;" << MAVProxyPath << " & ";
#else
    command << "export PATH=$PATH:" << MAVProxyPath << " & ";
#endif

    if(!shellCommand.empty()) {
        command << shellCommand << " \"";
    }
    command << simulatorPath << " -N -v ArduCopter -I " << +systemId << " --sysid " << +systemId << " --out 127.0.0.1:" << connectionPort << " & ";;
    if(!shellCommand.empty()) {
        command << "\"";
    }
    std::cout << command.str() << std::endl;
    std::system(command.str().c_str());
}

void MAVLinkManager::registerVehicle(IMAVLinkVehicle *vehicle, uint8_t systemId, uint8_t componentId) {
    EV_INFO << "Registering vehicle with sysid: " << +systemId << " and componentid: " << +componentId << std::endl;
    registeredVehicles.insert({VehicleEntry { systemId, componentId }, vehicle});
    startSimulator(systemId);
}

bool MAVLinkManager::sendMessage(const mavlink_message_t& message, uint8_t destinationId) {
    Enter_Method_Silent();
    int length = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);
    static const int c = sizeof(sockaddr_in);

    if(addressMap.find(destinationId) != addressMap.end()) {
        if(::sendto(fd, buf, length, 0, (sockaddr*) &addressMap[destinationId],c) == SOCKET_ERROR) {
            EV_ERROR << "Error sending message: " << WSAGetLastError() << std::endl;
            return false;
        }
        return true;
    }
    return false;
}


void MAVLinkManager::openSocket()
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
    if((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP )) == INVALID_SOCKET)
    {
        EV_ERROR << "Could not create socket : " << WSAGetLastError() << std::endl;
        return;
    }

    EV_INFO << "Socket created" << std::endl;

    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( connectionPort );

    //Bind
    if( bind(fd ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
    {
        EV_ERROR << "Bind failed with error code : " << WSAGetLastError() << std::endl;
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
    if(incoming == fd) {
        int length;
        struct sockaddr_in client;
        static int c = sizeof(struct sockaddr_in);

        do {
            if((length = recvfrom(fd, buf, 256, 0, (struct sockaddr *)&client, &c)) == SOCKET_ERROR) {
                int error = WSAGetLastError();
                EV_DETAIL << "Received WSAEWOULDBLOCK, the socket is empty." << std::endl;
                // Blocking errors are normal when no messages are present in a non-blocking socket
                if(error == WSAEWOULDBLOCK) return true;
                EV_ERROR << "Error receiving message, code: " << error << std::endl;
                return false;
            }
            EV_DEBUG << "Received " << length << " bytes." << std::endl;

            mavlink_status_t status;
            mavlink_message_t msg;

            for (int i = 0; i < length; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
                {
                    addressMap[msg.sysid] = client;

                    MAVLinkManager::IMAVLinkVehicle* vehicle = nullptr;
                    try {
                        vehicle = registeredVehicles.at(VehicleEntry {msg.sysid, msg.compid});
                    } catch(std::out_of_range error) {
                        EV_DEBUG << "Received message from unregistered vehicle ID: " << msg.sysid << " COMPID: " << msg.compid << std::endl;
                        break;
                    }
                    vehicle->receiveTelemetry(msg);
                }
            }
        } while(length > 0);
        return true;
    }
    return false;
}


MAVLinkManager::~MAVLinkManager() {
    WSACleanup();
    rtScheduler->removeCallback(fd, this);
    closesocket(fd);
}


}

