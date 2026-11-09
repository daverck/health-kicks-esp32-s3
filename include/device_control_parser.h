#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

class HealthKicksBleServer;

/**
 * @file device_control_parser.h
 * @brief ASCII command parser and validator for Device & Studio Control (Characteristic 0004).
 */
class DeviceControlParser {
public:
    /**
     * @brief Parses an ASCII control command buffer, validates arguments/ranges, and executes callbacks.
     * @param server Pointer to active HealthKicksBleServer instance.
     * @param data Pointer to raw byte buffer received over BLE.
     * @param length Number of bytes received.
     */
    static void parseAndExecute(HealthKicksBleServer* server, const uint8_t* data, size_t length);

private:
    static bool handleInactivityConfig(HealthKicksBleServer* server, const char* pCmd);
    static bool handleHitlConfig(HealthKicksBleServer* server, const char* pCmd);
    static bool handleFallConfidenceConfig(HealthKicksBleServer* server, const char* pCmd);
    static bool handleFallCooldownConfig(HealthKicksBleServer* server, const char* pCmd);
    static bool handleCalibration(HealthKicksBleServer* server, const char* pCmd);
};
