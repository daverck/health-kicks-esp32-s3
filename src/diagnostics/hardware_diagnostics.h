#pragma once

#include <Arduino.h>
#include "sensors/imu_mpu6050.h"
#include "actuators/haptic_driver.h"
#include "actuators/led_driver.h"

/**
 * @brief Self-test diagnostic suite executed during ESP32-S3 boot.
 */
class HardwareDiagnostics {
public:
    static bool runDiagnostics(ImuMpu6050& imu, HapticDriver& haptic, LedDriver& statusLed);
};
