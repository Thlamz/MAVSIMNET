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

template <VehicleType V>
std::vector<Instruction> armTakeoff(double altitude, uint8_t targetSystem, uint8_t targetComponent);

}
}



#endif /* MAVSIMNET_UTILS_VEHICLEROUTINES_H_ */
