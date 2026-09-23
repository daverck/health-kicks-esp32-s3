#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_sleep.h"
#include "config.h"
#include "invariants.h"

// Forward declarations
class ImuMpu6050;
class HealthKicksBleServer;
class HapticDriver;
class StepDetector;
class ImuCalibrator;

/**
 * @file power_manager.h
 * @brief Ultra-low power Deep Sleep and Wakeup Manager for ESP32-S3.
 * 
 * Manages inactivity evaluation, RTC Fast Memory state preservation,
 * MPU-6050 Wake-On-Motion (WOM) interrupt on GPIO 6, and manual switch wakeup on GPIO 14.
 */
class PowerManager {
public:
    /**
     * @brief Initializes power management, checks wakeup cause, and restores state from RTC fast memory.
     * @param stepDetector Pointer to StepDetector instance for counter restoration.
     * @param imuCalibrator Pointer to ImuCalibrator instance for alignment matrix restoration.
     */
    static void init(StepDetector* stepDetector, ImuCalibrator* imuCalibrator);

    /**
     * @brief Updates the last activity timestamp (resets inactivity countdown).
     * @param nowMs Current system timestamp in ms.
     */
    static void recordActivity(uint32_t nowMs);

    /**
     * @brief Checks whether the system meets Deep Sleep conditions (inactivity timeout, BLE disconnected, Studio idle).
     * @param isBleConnected True if a BLE client is currently connected.
     * @param isRecording True if a Studio recording session is currently running.
     * @param nowMs Current system timestamp in ms.
     * @param imu Reference to IMU driver.
     * @param bleServer Reference to BLE server.
     * @param haptic Reference to haptic driver.
     * @param stepDetector Reference to step detector.
     * @param imuCalibrator Reference to IMU calibrator.
     */
    static void checkSleepConditions(bool isBleConnected,
                                    bool isRecording,
                                    uint32_t nowMs,
                                    ImuMpu6050& imu,
                                    HealthKicksBleServer& bleServer,
                                    HapticDriver& haptic,
                                    StepDetector& stepDetector,
                                    ImuCalibrator& imuCalibrator);

    /**
     * @brief Executes clean shutdown pipeline and enters Deep Sleep (~10 uA).
     * @param imu Reference to IMU driver.
     * @param bleServer Reference to BLE server.
     * @param haptic Reference to haptic driver.
     * @param stepDetector Reference to step detector.
     * @param imuCalibrator Reference to IMU calibrator.
     */
    static void enterDeepSleep(ImuMpu6050& imu,
                              HealthKicksBleServer& bleServer,
                              HapticDriver& haptic,
                              StepDetector& stepDetector,
                              ImuCalibrator& imuCalibrator);

    /**
     * @brief Returns total boot count across Deep Sleep cycles.
     */
    static uint32_t getBootCount();

    /**
     * @brief Returns wakeup cause recorded during startup.
     */
    static esp_sleep_wakeup_cause_t getWakeupCause();

    /**
     * @brief Sets inactivity timeout in seconds (useful for test suites or runtime adjustments).
     */
    static void setTimeoutSec(uint32_t sec);

    /**
     * @brief Gets current inactivity timeout in seconds.
     */
    static uint32_t getTimeoutSec();

private:
    static uint32_t _lastActivityMs;
    static uint32_t _inactivityTimeoutSec;
    static esp_sleep_wakeup_cause_t _wakeupCause;
};
