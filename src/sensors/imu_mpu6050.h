#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "invariants.h"

class ImuMpu6050 {
public:
    ImuMpu6050();

    /**
     * @brief Initialise le bus I2C et configure les registres du MPU-6050.
     * @return true si l'IMU répond correctement (WHO_AM_I == 0x68), false sinon.
     */
    bool begin(int sdaPin = PIN_IMU_SDA, int sclPin = PIN_IMU_SCL, uint32_t frequency = IMU_I2C_FREQ_HZ);

    /**
     * @brief Vérifie la présence physique et lit le registre WHO_AM_I (0x75).
     * @return Valeur du registre WHO_AM_I (0x68 attendu).
     */
    uint8_t readWhoAmI();

    /**
     * @brief Lit 14 octets consécutifs du MPU6050 à partir du registre 0x3B.
     * @param frame Référence vers la structure de trame compressée au format HealthKicks.
     * @param deltaMs Horodatage relatif en ms depuis le début de capture.
     * @return true si la lecture I2C a réussi, false sinon.
     */
    bool readFrame(ImuRawFrame& frame, uint16_t deltaMs = 0);

    /**
     * @brief Lecture des valeurs physiques flottantes (milli-g et deg/s) pour affichage/debug.
     */
    bool readRawMetrics(float& ax_g, float& ay_g, float& az_g, float& gx_dps, float& gy_dps, float& gz_dps);

private:
    uint8_t _address;
    TwoWire* _wire;

    bool readAlignedRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

