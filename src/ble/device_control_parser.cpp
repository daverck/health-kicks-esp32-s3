#include "device_control_parser.h"
#include "ble_server.h"
#include "invariants.h"
#include <string.h>

bool DeviceControlParser::handleInactivityConfig(HealthKicksBleServer* server, const char* pCmd) {
    if (strncmp(pCmd, "SET_INACTIVITY", 14) != 0) return false;
    int enabledInt = 0;
    unsigned int thresh = 0;
    unsigned int cool = 0;
    if (sscanf(pCmd + 14, "%d %u %u", &enabledInt, &thresh, &cool) == 3) {
        bool enabled = (enabledInt != 0);
        Serial.printf("[BLE] Device Control SET_INACTIVITY: enabled=%d, thresh=%u s, cool=%u s\n",
                      enabled, (uint16_t)thresh, (uint16_t)cool);
        if (server->_onInactivityConfig) {
            server->_onInactivityConfig(enabled, (uint16_t)thresh, (uint16_t)cool);
        }
        server->notifyStudioControl("INACTIVITY_CONFIG_OK");
    } else {
        server->notifyStudioControl("INACTIVITY_CONFIG_ERROR invalid_params");
    }
    return true;
}

bool DeviceControlParser::handleHitlConfig(HealthKicksBleServer* server, const char* pCmd) {
    if (strncmp(pCmd, "SET_HITL", 8) != 0) return false;
    int val = -1;
    if (sscanf(pCmd + 8, "%d", &val) == 1 && (val == 0 || val == 1)) {
        server->_hitlEnabled = (val == 1);
        Serial.printf("[BLE] Device Control SET_HITL: %d\n", val);
        if (server->_onHitlConfig) {
            server->_onHitlConfig(server->_hitlEnabled);
        }
        char resp[32];
        snprintf(resp, sizeof(resp), "HITL_OK %d", val);
        server->notifyStudioControl(resp);
    } else {
        server->notifyStudioControl("HITL_ERROR invalid_params");
    }
    return true;
}

bool DeviceControlParser::handleFallConfidenceConfig(HealthKicksBleServer* server, const char* pCmd) {
    if (strncmp(pCmd, "SET_FALL_CONFIDENCE", 19) != 0) return false;
    float threshold = 0.0f;
    if (sscanf(pCmd + 19, "%f", &threshold) == 1) {
        if (threshold >= MIN_FALL_CONFIDENCE_THRESHOLD && threshold <= MAX_FALL_CONFIDENCE_THRESHOLD) {
            Serial.printf("[BLE] Device Control SET_FALL_CONFIDENCE: %.2f\n", threshold);
            if (server->_onFallConfidenceConfig) {
                server->_onFallConfidenceConfig(threshold);
            }
            char resp[32];
            snprintf(resp, sizeof(resp), "FALL_CONFIDENCE_OK %.2f", threshold);
            server->notifyStudioControl(resp);
        } else {
            server->notifyStudioControl("FALL_CONFIDENCE_ERROR out_of_range");
        }
    } else {
        server->notifyStudioControl("FALL_CONFIDENCE_ERROR invalid_params");
    }
    return true;
}

bool DeviceControlParser::handleFallCooldownConfig(HealthKicksBleServer* server, const char* pCmd) {
    if (strncmp(pCmd, "SET_FALL_COOLDOWN", 17) != 0) return false;
    unsigned int cooldownMs = 0;
    if (sscanf(pCmd + 17, "%u", &cooldownMs) == 1) {
        if (cooldownMs >= MIN_FALL_COOLDOWN_MS && cooldownMs <= MAX_FALL_COOLDOWN_MS) {
            Serial.printf("[BLE] Device Control SET_FALL_COOLDOWN: %u ms\n", cooldownMs);
            if (server->_onFallCooldownConfig) {
                server->_onFallCooldownConfig((uint32_t)cooldownMs);
            }
            char resp[32];
            snprintf(resp, sizeof(resp), "FALL_COOLDOWN_OK %u", cooldownMs);
            server->notifyStudioControl(resp);
        } else {
            server->notifyStudioControl("FALL_COOLDOWN_ERROR out_of_range");
        }
    } else {
        server->notifyStudioControl("FALL_COOLDOWN_ERROR invalid_params");
    }
    return true;
}

bool DeviceControlParser::handleCalibration(HealthKicksBleServer* server, const char* pCmd) {
    if (strcasecmp(pCmd, "CALIBRATE") == 0 || strncasecmp(pCmd, "CALIB", 5) == 0) {
        Serial.println("[BLE] Device Control CALIBRATE command received");
        if (server->_onCalibration) {
            server->_onCalibration();
        }
        return true;
    }
    return false;
}

void DeviceControlParser::parseAndExecute(HealthKicksBleServer* server, const uint8_t* data, size_t length) {
    if (!server || length == 0) return;

    // Fixed stack buffer avoiding dynamic heap String allocations and fragmentation
    char cmdBuf[128];
    size_t copyLen = (length < sizeof(cmdBuf) - 1) ? length : (sizeof(cmdBuf) - 1);
    memcpy(cmdBuf, data, copyLen);
    cmdBuf[copyLen] = '\0';

    // In-place trim of trailing control/whitespace characters
    while (copyLen > 0 && (cmdBuf[copyLen - 1] == ' ' || cmdBuf[copyLen - 1] == '\r' || cmdBuf[copyLen - 1] == '\n' || cmdBuf[copyLen - 1] == '\t')) {
        cmdBuf[--copyLen] = '\0';
    }

    // Skip leading whitespace
    const char* pCmd = cmdBuf;
    while (*pCmd == ' ' || *pCmd == '\t') {
        pCmd++;
    }

    if (*pCmd == '\0') return;

    log_i("BLE Device & Studio Control Write received: \"%s\"", pCmd);

    if (handleInactivityConfig(server, pCmd)) return;
    if (handleHitlConfig(server, pCmd)) return;
    if (handleFallConfidenceConfig(server, pCmd)) return;
    if (handleFallCooldownConfig(server, pCmd)) return;
    if (handleCalibration(server, pCmd)) return;

    // Studio commands (START, FINISH, CANCEL) dispatched via callback
    if (server->_onStudioCommand) {
        server->_onStudioCommand(std::string(pCmd));
    }
}
