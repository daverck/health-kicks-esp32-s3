#include "haptic_driver.h"
#include <Arduino.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>

#define HAPTIC_CHANNEL 4
#define HAPTIC_FREQ    10000
#define HAPTIC_RES     8

static esp_timer_handle_t s_haptic_timer = nullptr;

static void IRAM_ATTR haptic_timer_callback(void* arg) {
    ledcWrite(HAPTIC_CHANNEL, 0);
    ets_printf("[HAPTIC] Extinction materielle PWM via timer\n");
}

HapticDriver::HapticDriver()
    : _pin(PIN_HAPTIC_PWM),
      _pattern(HAPTIC_PATTERN_CONTINUOUS),
      _currentIntensity(0),
      _stopTimestampMs(0),
      _isActive(false),
      _patternStep(0),
      _stepTime(0) {}

void HapticDriver::init(int pin) {
    _pin = pin;
    ledcSetup(HAPTIC_CHANNEL, HAPTIC_FREQ, HAPTIC_RES);
    ledcAttachPin(_pin, HAPTIC_CHANNEL);
    ledcWrite(HAPTIC_CHANNEL, 0);

    const esp_timer_create_args_t timer_args = {
        .callback = &haptic_timer_callback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "haptic_stop_timer"
    };
    if (s_haptic_timer == nullptr) {
        esp_timer_create(&timer_args, &s_haptic_timer);
    }
    Serial.printf("[HAPTIC] Initialise sur GPIO %d, canal LEDC %d avec timer dedie\n", _pin, HAPTIC_CHANNEL);
}

void HapticDriver::setIntensity(uint8_t intensity) {
    ledcWrite(HAPTIC_CHANNEL, intensity);
}

void HapticDriver::play(uint8_t pattern, uint8_t intensity, uint16_t duration_ms) {
    if (intensity == 0 || duration_ms == 0) {
        stop();
        return;
    }
    if (s_haptic_timer) {
        esp_timer_stop(s_haptic_timer);
    }

    ledcWrite(HAPTIC_CHANNEL, intensity);
    Serial.printf("[HAPTIC] PLAY: int=%d, dur=%d ms\n", intensity, duration_ms);

    if (s_haptic_timer) {
        esp_timer_start_once(s_haptic_timer, (uint64_t)duration_ms * 1000ULL);
    }
}

void HapticDriver::stop() {
    if (s_haptic_timer) {
        esp_timer_stop(s_haptic_timer);
    }
    ledcWrite(HAPTIC_CHANNEL, 0);
}

void HapticDriver::update() {
    // La gestion d'arret est desormais 100% asynchrone et confiee a l'esp_timer.
}

void HapticDriver::testRampUp() {
    Serial.println("--- Démarrage Test Ramp-Up Vibreur Haptique (0 -> 255) ---");
    for (int i = 0; i <= 255; i += 15) {
        setIntensity(i);
        delay(30);
    }
    delay(200);
    for (int i = 255; i >= 0; i -= 15) {
        setIntensity(i);
        delay(30);
    }
    stop();
    Serial.println("--- Fin Test Ramp-Up Vibreur Haptique ---");
}
