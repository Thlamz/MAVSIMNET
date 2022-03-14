#include "mavlink/ardupilotmega/mavlink.h"
#include <functional>

namespace mavsimnet {
namespace TelemetryConditions {
    bool checkEmpty(mavlink_message_t);

    std::function<bool(mavlink_message_t)> getCheckCmdAck(uint8_t systemId, uint8_t componentId, unsigned short command, uint8_t senderSystemId);

    std::function<bool(mavlink_message_t)> getCheckPreArm(uint8_t senderSystemId);

    std::function<bool(mavlink_message_t)> getCheckArm(uint8_t senderSystemId);

    std::function<bool(mavlink_message_t)> getCheckAltitude(int32_t altitude, int32_t tolerance, uint8_t senderSystemId);

    std::function<bool(mavlink_message_t)> getCheckMissionRequest(uint8_t systemid, uint8_t componentId, uint16_t sequenceNumber, uint8_t senderSystemId);

    std::function<bool(mavlink_message_t)> getCheckMissionAck(uint8_t systemId, uint8_t componentId, uint8_t senderSystemId);

    std::function<bool(mavlink_message_t)> getCheckTargetGlobal(float lat, float lon, float alt, uint8_t senderSystemId);

    std::function<bool(mavlink_message_t)> getCheckParamValue(std::string param_id, float param_value, uint8_t senderSystemId);
}
}
