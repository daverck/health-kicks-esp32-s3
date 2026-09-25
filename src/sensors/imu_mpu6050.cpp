#include "imu_mpu6050.h"

// MPU-6050 Registers
static const uint8_t MPU_REG_SMPLRT_DIV      = 0x19;
static const uint8_t MPU_REG_CONFIG          = 0x1A;
static const uint8_t MPU_REG_GYRO_CONFIG     = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG    = 0x1C;
static const uint8_t MPU_REG_MOT_THR         = 0x1F;
static const uint8_t MPU_REG_MOT_DUR         = 0x20;
static const uint8_t MPU_REG_INT_PIN_CFG     = 0x37;
static const uint8_t MPU_REG_INT_ENABLE      = 0x38;
static const uint8_t MPU_REG_INT_STATUS      = 0x3A;
static const uint8_t MPU_REG_ACCEL_XOUT_H    = 0x3B;
static const uint8_t MPU_REG_MOT_DETECT_CTRL = 0x69;
static const uint8_t MPU_REG_PWR_MGMT_1      = 0x6B;
static const uint8_t MPU_REG_PWR_MGMT_2      = 0x6C;
static const uint8_t MPU_REG_WHO_AM_I        = 0x75;

ImuMpu6050::ImuMpu6050() : _address(IMU_I2C_ADDR), _wire(&Wire) {}

bool ImuMpu6050::begin(int sdaPin, int sclPin, uint32_t frequency) {
    _wire->begin(sdaPin, sclPin);
    _wire->setClock(frequency);
    _wire->setTimeOut(50);

    delay(50);

    // 1. Wake up from any previous sleep/cycle mode (Bit 6 SLEEP = 0, Bit 5 CYCLE = 0)
    writeRegister(MPU_REG_PWR_MGMT_1, 0x00);
    delay(10);

    // 2. Software reset of MPU-6050 (Bit 7 = 1)
    if (!writeRegister(MPU_REG_PWR_MGMT_1, 0x80)) {
        return false;
    }
    delay(100);

    // 3. Clear sleep mode after reset (reset sets Bit 6 SLEEP = 1)
    if (!writeRegister(MPU_REG_PWR_MGMT_1, 0x00)) {
        return false;
    }
    delay(20);

    // 4. Fully enable all accelerometer and gyroscope axes in PWR_MGMT_2 (clear standby modes)
    if (!writeRegister(MPU_REG_PWR_MGMT_2, 0x00)) {
        return false;
    }
    delay(10);

    // 5. Select auto X-Gyro clock source (0x01)
    if (!writeRegister(MPU_REG_PWR_MGMT_1, 0x01)) {
        return false;
    }
    delay(20);

    // 6. Disable all interrupts and reset INT pin config
    writeRegister(MPU_REG_INT_ENABLE, 0x00);
    writeRegister(MPU_REG_INT_PIN_CFG, 0x00);
    readRegister(MPU_REG_INT_STATUS); // Clear any latched status

    // 7. Verify WHO_AM_I
    uint8_t who = readWhoAmI();
    if (who != 0x68 && who != 0x70 && who != 0x72) { // 0x68 is nominal, some silicon revisions return 0x70/0x72
        log_e("Invalid IMU WHO_AM_I: 0x%02X (expected 0x68)", who);
        return false;
    }

    // 8. Configure accelerometer to +/- 8g (0x1C = 0x10) -> 4096 LSB/g, HPF disabled
    if (!writeRegister(MPU_REG_ACCEL_CONFIG, 0x10)) {
        return false;
    }

    // 9. Configure gyroscope to +/- 250 deg/s (0x1B = 0x00) -> 131.0 LSB/(deg/s)
    if (!writeRegister(MPU_REG_GYRO_CONFIG, 0x00)) {
        return false;
    }

    // 10. Configure hardware low-pass filter (DLPF ~21 Hz) (0x1A = 0x03)
    if (!writeRegister(MPU_REG_CONFIG, 0x03)) {
        return false;
    }

    // 11. Sample Rate Divider (Sample Rate Divider = 19 -> 50 Hz with 1 kHz DLPF base)
    // Sample Rate = 1000 / (1 + 19) = 50 Hz
    if (!writeRegister(MPU_REG_SMPLRT_DIV, 19)) {
        return false;
    }

    log_i("MPU-6050 initialized successfully on I2C (SDA=%d, SCL=%d, WHO=0x%02X)", sdaPin, sclPin, who);
    return true;
}

uint8_t ImuMpu6050::readWhoAmI() {
    return readRegister(MPU_REG_WHO_AM_I);
}

bool ImuMpu6050::readAlignedRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
    _wire->beginTransmission(_address);
    _wire->write(MPU_REG_ACCEL_XOUT_H);
    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    if (_wire->requestFrom(_address, (size_t)14, true) != 14) {
        return false;
    }

    // Raw chip axes reading
    int16_t chip_ax = (_wire->read() << 8) | _wire->read();
    int16_t chip_ay = (_wire->read() << 8) | _wire->read();
    int16_t chip_az = (_wire->read() << 8) | _wire->read();
    _wire->read(); _wire->read(); // Ignore temperature bytes
    int16_t chip_gx = (_wire->read() << 8) | _wire->read();
    int16_t chip_gy = (_wire->read() << 8) | _wire->read();
    int16_t chip_gz = (_wire->read() << 8) | _wire->read();

    // Standard Footwear Reference Frame Transformation:
    // X (Forward) = -chip_ay
    // Y (Left)    = +chip_ax
    // Z (Up)      = +chip_az
    ax = -chip_ay;
    ay = chip_ax;
    az = chip_az;

    // Apply the same right-handed rotation matrix to angular velocities
    gx = -chip_gy;
    gy = chip_gx;
    gz = chip_gz;

    return true;
}

bool ImuMpu6050::readFrame(ImuRawFrame& frame, uint16_t deltaMs) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readAlignedRaw(ax, ay, az, gx, gy, gz)) {
        return false;
    }

    // Scale factors matching contracts/ble_gatt_specs.md:
    // - Acceleration: milli-g (x1000) from +/- 8g scale (4096 LSB/g)
    // - Gyroscope: tenths of deg/s (x10) from +/- 250 dps scale (131.0 LSB/dps)
    int16_t milli_ax = (int16_t)(((int32_t)ax * 1000) / 4096);
    int16_t milli_ay = (int16_t)(((int32_t)ay * 1000) / 4096);
    int16_t milli_az = (int16_t)(((int32_t)az * 1000) / 4096);

    int16_t dixieme_gx = (int16_t)(((int32_t)gx * 100) / 1310);
    int16_t dixieme_gy = (int16_t)(((int32_t)gy * 100) / 1310);
    int16_t dixieme_gz = (int16_t)(((int32_t)gz * 100) / 1310);

    // Big-Endian network encoding for BLE packet transmission
    frame.delta_ms = htons(deltaMs);
    frame.ax = htons(milli_ax);
    frame.ay = htons(milli_ay);
    frame.az = htons(milli_az);
    frame.gx = htons(dixieme_gx);
    frame.gy = htons(dixieme_gy);
    frame.gz = htons(dixieme_gz);

    return true;
}

bool ImuMpu6050::readRawMetrics(float& ax_g, float& ay_g, float& az_g, float& gx_dps, float& gy_dps, float& gz_dps) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readAlignedRaw(ax, ay, az, gx, gy, gz)) {
        return false;
    }

    // Convert raw aligned LSB values into standard physical float units (g and deg/s)
    ax_g = (float)ax / 4096.0f;
    ay_g = (float)ay / 4096.0f;
    az_g = (float)az / 4096.0f;

    gx_dps = (float)gx / 131.0f;
    gy_dps = (float)gy / 131.0f;
    gz_dps = (float)gz / 131.0f;

    return true;
}

bool ImuMpu6050::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    return (_wire->endTransmission(true) == 0);
}

uint8_t ImuMpu6050::readRegister(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
        return 0;
    }
    if (_wire->requestFrom(_address, (size_t)1, true) == 1) {
        return _wire->read();
    }
    return 0;
}

bool ImuMpu6050::enableWakeOnMotion(uint8_t threshold, uint8_t duration) {
    // 1. Ensure chip is awake to configure registers
    if (!writeRegister(MPU_REG_PWR_MGMT_1, 0x00)) {
        return false;
    }
    delay(10);

    // 2. Configure INT pin: Active HIGH, Push-Pull, Latch until cleared
    if (!writeRegister(MPU_REG_INT_PIN_CFG, 0x20)) {
        return false;
    }

    // 3. Configure High-Pass Filter for motion detection (5Hz HPF, +/-8g scale)
    if (!writeRegister(MPU_REG_ACCEL_CONFIG, 0x11)) {
        return false;
    }

    // 4. Set motion detection threshold and duration
    if (!writeRegister(MPU_REG_MOT_THR, threshold)) {
        return false;
    }
    if (!writeRegister(MPU_REG_MOT_DUR, duration)) {
        return false;
    }

    // 5. Configure motion detection control (accel power-on delay)
    if (!writeRegister(MPU_REG_MOT_DETECT_CTRL, 0x15)) {
        return false;
    }

    // 6. Enable Motion Detection Interrupt (Bit 6 MOT_EN = 0x40)
    if (!writeRegister(MPU_REG_INT_ENABLE, 0x40)) {
        return false;
    }

    // 7. Configure low-power sampling in PWR_MGMT_2:
    // LP_WAKE_CTRL = 00 (1.25 Hz), STBY_ACCEL = 000 (active), STBY_GYRO = 111 (disable gyroscopes to minimize current)
    if (!writeRegister(MPU_REG_PWR_MGMT_2, 0x07)) {
        return false;
    }

    // 8. Put MPU-6050 in CYCLE mode (Bit 5 CYCLE = 1, Bit 6 SLEEP = 0, Bit 3 TEMP_DIS = 1) -> 0x28
    if (!writeRegister(MPU_REG_PWR_MGMT_1, 0x28)) {
        return false;
    }

    log_i("MPU-6050 configured for Wake-On-Motion (WOM): Thr=%u, Dur=%u, 1.25 Hz cycle mode", threshold, duration);
    return true;
}
