#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "invariants.h"

class ImuMpu6050 {
public:
    ImuMpu6050();

    /**
     * @brief Initializes the I2C bus and configures MPU-6050 hardware registers.
     * @param sdaPin I2C data pin (default: PIN_IMU_SDA).
     * @param sclPin I2C clock pin (default: PIN_IMU_SCL).
     * @param frequency I2C bus frequency in Hz (default: IMU_I2C_FREQ_HZ).
     * @return true if the IMU responds with expected WHO_AM_I, false otherwise.
     */
    bool begin(int sdaPin = PIN_IMU_SDA, int sclPin = PIN_IMU_SCL, uint32_t frequency = IMU_I2C_FREQ_HZ);

    /**
     * @brief Verifies physical chip presence by reading the WHO_AM_I register (0x75).
     * @return Value of WHO_AM_I register (0x68 expected nominally).
     */
    uint8_t readWhoAmI();

    /**
     * @brief Reads 14 consecutive burst bytes from MPU-6050 starting at register 0x3B.
     * Converts chip raw coordinates into standard footwear frame and scales to network units.
     * @param frame Reference to the compressed raw frame structure in big-endian network order.
     * @param deltaMs Relative timestamp in milliseconds since session capture start.
     * @return true if the I2C transaction succeeded, false on bus error.
     */
    bool readFrame(ImuRawFrame& frame, uint16_t deltaMs = 0);

    /**
     * @brief Reads aligned acceleration and angular velocity in physical engineering float units.
     * @param ax_g Output acceleration X in g.
     * @param ay_g Output acceleration Y in g.
     * @param az_g Output acceleration Z in g.
     * @param gx_dps Output angular rate X in deg/s.
     * @param gy_dps Output angular rate Y in deg/s.
     * @param gz_dps Output angular rate Z in deg/s.
     * @return true on successful I2C read, false otherwise.
     */
    bool readRawMetrics(float& ax_g, float& ay_g, float& az_g, float& gx_dps, float& gy_dps, float& gz_dps);

    /**
     * @brief Configures MPU-6050 in ultra-low power Wake-On-Motion (WOM) mode for Deep Sleep.
     * @param threshold Acceleration threshold (1 LSB = 32 mg, default from config.h).
     * @param duration Minimum event duration count (default from config.h).
     * @return true if registers configured successfully, false otherwise.
     */
    bool enableWakeOnMotion(uint8_t threshold = IMU_WOM_THRESHOLD, uint8_t duration = IMU_WOM_DURATION);

    /**
     * @brief Puts MPU-6050 into low-power sleep mode (~5 uA) with interrupts disabled or wakes it up.
     * @param enable True to enter sleep mode, false to wake up.
     * @return true if register configuration was successful, false otherwise.
     */
    bool setSleepEnabled(bool enable);

private:
    uint8_t _address;
    TwoWire* _wire;

    bool readAlignedRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

