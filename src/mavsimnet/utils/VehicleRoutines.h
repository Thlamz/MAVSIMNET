/*
 * VehicleRoutines.h
 *
 *  Created on: 12 de mar de 2022
 *      Author: thlam
 */

#ifndef MAVSIMNET_UTILS_VEHICLEROUTINES_H_
#define MAVSIMNET_UTILS_VEHICLEROUTINES_H_

#include <vector>
#include <memory>
#include "VehicleTypes.h"
#include "mavsimnet/mobility/base/MAVLinkInstruction.h"
#include "inet/common/geometry/common/GeographicCoordinateSystem.h"

namespace mavsimnet {
namespace VehicleRoutines {

std::vector<std::shared_ptr<Instruction>> armTakeoff(uint8_t senderSystem, uint8_t senderComponent, VehicleType type, float altitude, uint8_t targetSystem, uint8_t targetComponent, omnetpp::simtime_t timeout=60, int retries=3);

enum Mode {
    GUIDED,
    AUTO,
    TAKEOFF
};
std::vector<std::shared_ptr<Instruction>> setMode(VehicleType type, Mode mode, uint8_t targetSystem, uint8_t targetComponent, omnetpp::simtime_t timeout=60, int retries=3);

/* GUIDED MODE COMMANDS */
std::vector<std::shared_ptr<Instruction>> guidedGoto(VehicleType type, double latitude, double longitude, float altitude, double tolerance,
        inet::IGeographicCoordinateSystem *coordinateSystem, uint8_t targetSystem, uint8_t targetComponent, omnetpp::simtime_t timeout=60, int retries=3);
}
}



#endif /* MAVSIMNET_UTILS_VEHICLEROUTINES_H_ */
