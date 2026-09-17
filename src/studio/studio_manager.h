#pragma once

#include <Arduino.h>
#include <vector>
#include <string>
#include "config.h"
#include "invariants.h"
#include "sensors/imu_mpu6050.h"
#include "actuators/haptic_driver.h"
#include "ble/ble_server.h"

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

    void begin(HealthKicksBleServer* bleServer, HapticDriver* haptic, ImuMpu6050* imu);

    /**
     * @brief Parse et traite une commande Studio Control (ex: "START walk 5.0 <uuid>" ou "CANCEL").
     */
    void handleCommand(const std::string& command);

    /**
     * @brief Boucle de mise à jour non bloquante appelée à chaque itération de loop().
     */
    void update();

    /**
     * @brief Annule la session Studio en cours.
     */
    void cancel();

    bool isRecording() const { return _state == StudioState::RECORDING; }
    bool isBusy() const { return _state != StudioState::IDLE; }

private:
    void startCountdown();
    void startRecording();
    void finishRecordingAndStreamBurst();
    void sendBurstPackets();

    HealthKicksBleServer* _bleServer;
    HapticDriver* _haptic;
    ImuMpu6050* _imu;

    StudioState _state;
    std::string _label;
    float _durationSec;
    std::string _sessionId;

    uint32_t _stepTimestampMs;
    uint32_t _recordingStartTimeMs;
    uint32_t _lastSampleTimeMs;

    std::vector<ImuRawFrame> _frames;
};
