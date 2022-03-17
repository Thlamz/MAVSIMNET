/*
 * MAVLinkInstruction.h
 *
 *  Created on: 12 de mar de 2022
 *      Author: thlamz
 */

#ifndef MAVSIMNET_MOBILITY_BASE_MAVLINKINSTRUCTION_H_
#define MAVSIMNET_MOBILITY_BASE_MAVLINKINSTRUCTION_H_

#include <omnetpp.h>
#include <functional>
#include "mavlink/ardupilotmega/mavlink.h"

namespace mavsimnet {

typedef std::function<bool(mavlink_message_t)> Condition;

struct Instruction {
    Instruction(mavlink_message_t message, Condition condition, omnetpp::simtime_t timeout, int retries, bool completed=false) : message(message), condition(condition), timeout(timeout), retries(retries), completed(completed) {};
    mavlink_message_t message;
    Condition condition;
    omnetpp::simtime_t timeout;
    int retries;
    bool completed = false;
};


}


#endif /* MAVSIMNET_MOBILITY_BASE_MAVLINKINSTRUCTION_H_ */
