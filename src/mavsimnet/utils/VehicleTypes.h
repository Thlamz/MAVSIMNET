/*
 * VehicleTypes.h
 *
 *  Created on: 12 de mar de 2022
 *      Author: thlam
 */

#ifndef MAVSIMNET_UTILS_VEHICLETYPES_H_
#define MAVSIMNET_UTILS_VEHICLETYPES_H_


namespace mavsimnet {

enum VehicleType: unsigned int {
    COPTER = 1,
    PLANE = 1<<1,
    ROVER = 1<<2
};

}


#endif /* MAVSIMNET_UTILS_VEHICLETYPES_H_ */
