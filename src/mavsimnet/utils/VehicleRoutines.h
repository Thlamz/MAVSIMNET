/*
 * VehicleRoutines.h
 *
 *  Created on: 12 de mar de 2022
 *      Author: thlam
 */

#ifndef MAVSIMNET_UTILS_VEHICLEROUTINES_H_
#define MAVSIMNET_UTILS_VEHICLEROUTINES_H_

#include <vector>
#include "VehicleTypes.h"
#include "mavsimnet/mobility/base/MAVLinkInstruction.h"

namespace mavsimnet {
namespace VehicleRoutines {

std::vector<Instruction> armTakeoff(VehicleType type, float altitude, uint8_t targetSystem, uint8_t targetComponent);

enum Mode {
    GUIDED,
    AUTO,
    TAKEOFF
};
std::vector<Instruction> setMode(VehicleType type, Mode mode, uint8_t targetSystem, uint8_t targetComponent);

/* GUIDED MODE COMMANDS */
std::vector<Instruction> guidedGoto(VehicleType type, double latitude, double longitude, float altitude, uint8_t targetSystem, uint8_t targetComponent);
}
}



#endif /* MAVSIMNET_UTILS_VEHICLEROUTINES_H_ */
