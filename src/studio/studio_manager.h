#pragma once

#include <Arduino.h>
#include <vector>
#include <string>
#include "config.h"
#include "invariants.h"
#include "sensors/imu_mpu6050.h"
#include "actuators/haptic_driver.h"
#include "ble/ble_server.h"
#include "imu_calibrator.h"

enum class StudioState {
    IDLE,
    COUNTDOWN_PULSE_1,
    COUNTDOWN_PULSE_2,
    COUNTDOWN_PULSE_3,
    RECORDING,
    TRANSMITTING_BURST
};

class StudioManager {
public:
    StudioManager();

    void begin(HealthKicksBleServer* bleServer, HapticDriver* haptic, ImuMpu6050* imu, ImuCalibrator* calibrator = nullptr);

    /**
     * @brief Parses and processes a Studio Control command (e.g. "START walk 5.0 <uuid>" or "CANCEL").
     */
    void handleCommand(const std::string& command);

    /**
     * @brief Non-blocking update loop called at each loop() iteration.
     */
    void update();

    /**
     * @brief Cancels current Studio session.
     */
    void cancel();

    bool isRecording() const { return _state == StudioState::RECORDING; }
    bool isBusy() const { return _state != StudioState::IDLE; }

    /**
     * @brief Streams a burst of IMU frames acquired in Human-in-the-Loop (HITL) mode.
     */
    void streamHitlBurst(const std::vector<ImuRawFrame>& frames, const std::string& sessionId);

private:
    void handleStartCommand(const std::string& command);
    void handleFinishCommand();
    void handleCancelCommand();

    void startCountdown();
    void triggerCountdownStep(StudioState nextState, const char* stepStr);
    void startRecording();
    void finishRecordingAndStreamBurst();
    void sendBurstPackets();

    HealthKicksBleServer* _bleServer;
    HapticDriver* _haptic;
    ImuMpu6050* _imu;
    ImuCalibrator* _calibrator;

    StudioState _state;
    std::string _label;
    float _durationSec;
    std::string _sessionId;

    uint32_t _stepTimestampMs;
    uint32_t _recordingStartTimeMs;
    uint32_t _lastSampleTimeMs;
    volatile bool _finishRequested;

    std::vector<ImuRawFrame> _frames;
};
