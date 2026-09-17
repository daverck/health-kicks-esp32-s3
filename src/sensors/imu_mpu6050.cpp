#include "imu_mpu6050.h"

// Registres MPU-6050
static const uint8_t MPU_REG_SMPLRT_DIV   = 0x19;
static const uint8_t MPU_REG_CONFIG       = 0x1A;
static const uint8_t MPU_REG_GYRO_CONFIG  = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;
static const uint8_t MPU_REG_WHO_AM_I     = 0x75;

ImuMpu6050::ImuMpu6050() : _address(IMU_I2C_ADDR), _wire(&Wire) {}

bool ImuMpu6050::begin(int sdaPin, int sclPin, uint32_t frequency) {
    _wire->begin(sdaPin, sclPin);
    _wire->setClock(frequency);

    delay(50);

    // 1. Reset logiciel du MPU-6050 (Bit 7 = 1)
    if (!writeRegister(MPU_REG_PWR_MGMT_1, 0x80)) {
        return false;
    }
    delay(100);

    // 2. Sortie de veille + Sélection source horloge Auto X-Gyro (0x01)
    if (!writeRegister(MPU_REG_PWR_MGMT_1, 0x01)) {
        return false;
    }
    delay(30);

    // 3. Vérification WHO_AM_I
    uint8_t who = readWhoAmI();
    if (who != 0x68 && who != 0x70 && who != 0x72) { // 0x68 est nominal, certaines révisions renvoient 0x70/0x72
        log_e("IMU WHO_AM_I invalide: 0x%02X (attendu 0x68)", who);
        return false;
    }

    // 4. Configuration accéléromètre à +/- 8g (0x1C = 0x10) -> 4096 LSB/g
    if (!writeRegister(MPU_REG_ACCEL_CONFIG, 0x10)) {
        return false;
    }

    // 5. Configuration gyroscope à +/- 250 deg/s (0x1B = 0x00) -> 131.0 LSB/(deg/s)
    if (!writeRegister(MPU_REG_GYRO_CONFIG, 0x00)) {
        return false;
    }

    // 6. Configuration du filtre passe-bas matériel (DLPF ~21 Hz) (0x1A = 0x03)
    if (!writeRegister(MPU_REG_CONFIG, 0x03)) {
        return false;
    }

    // 7. Diviseur de cadence (Sample Rate Divider = 0x01 -> 50 Hz avec DLPF 1 kHz base)
    // Sample Rate = 1000 / (1 + 19) = 50 Hz
    if (!writeRegister(MPU_REG_SMPLRT_DIV, 19)) {
        return false;
    }

    log_i("MPU-6050 initialisé avec succès sur I2C (SDA=%d, SCL=%d, WHO=0x%02X)", sdaPin, sclPin, who);
    return true;
}

uint8_t ImuMpu6050::readWhoAmI() {
    return readRegister(MPU_REG_WHO_AM_I);
}

bool ImuMpu6050::readFrame(ImuRawFrame& frame, uint16_t deltaMs) {
    _wire->beginTransmission(_address);
    _wire->write(MPU_REG_ACCEL_XOUT_H);
    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    if (_wire->requestFrom(_address, (size_t)14, true) != 14) {
        return false;
    }

    int16_t raw_ax = (_wire->read() << 8) | _wire->read();
    int16_t raw_ay = (_wire->read() << 8) | _wire->read();
    int16_t raw_az = (_wire->read() << 8) | _wire->read();
    _wire->read(); _wire->read(); // Ignore temperature bytes
    int16_t raw_gx = (_wire->read() << 8) | _wire->read();
    int16_t raw_gy = (_wire->read() << 8) | _wire->read();
    int16_t raw_gz = (_wire->read() << 8) | _wire->read();

    // Facteurs d'échelle conformes au contrat contracts/ble_gatt_specs.md :
    // - Accélération : milli-g (x1000) depuis échelle +/- 8g (4096 LSB/g)
    // - Gyroscope : dixièmes de deg/s (x10) depuis échelle +/- 250 dps (131.0 LSB/dps)
    int16_t milli_ax = (int16_t)(((int32_t)raw_ax * 1000) / 4096);
    int16_t milli_ay = (int16_t)(((int32_t)raw_ay * 1000) / 4096);
    int16_t milli_az = (int16_t)(((int32_t)raw_az * 1000) / 4096);

    int16_t dixieme_gx = (int16_t)(((int32_t)raw_gx * 100) / 1310);
    int16_t dixieme_gy = (int16_t)(((int32_t)raw_gy * 100) / 1310);
    int16_t dixieme_gz = (int16_t)(((int32_t)raw_gz * 100) / 1310);

    // Encodage réseau Big-Endian pour le transport BLE
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
    _wire->beginTransmission(_address);
    _wire->write(MPU_REG_ACCEL_XOUT_H);
    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    if (_wire->requestFrom(_address, (size_t)14, true) != 14) {
        return false;
    }

    int16_t raw_ax = (_wire->read() << 8) | _wire->read();
    int16_t raw_ay = (_wire->read() << 8) | _wire->read();
    int16_t raw_az = (_wire->read() << 8) | _wire->read();
    _wire->read(); _wire->read();
    int16_t raw_gx = (_wire->read() << 8) | _wire->read();
    int16_t raw_gy = (_wire->read() << 8) | _wire->read();
    int16_t raw_gz = (_wire->read() << 8) | _wire->read();

    ax_g = (float)raw_ax / 4096.0f;
    ay_g = (float)raw_ay / 4096.0f;
    az_g = (float)raw_az / 4096.0f;
    gx_dps = (float)raw_gx / 131.0f;
    gy_dps = (float)raw_gy / 131.0f;
    gz_dps = (float)raw_gz / 131.0f;

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
