#include "ICM20948.h"

ICM20948::ICM20948()
{
    // コンストラクタ：変数を初期化
    accelX = accelY = accelZ = 0.0f;
    gyroX = gyroY = gyroZ = 0.0f;
    temperature = 0.0f;
    accelSensitivity = 1.0f;
    gyroSensitivity = 1.0f;
    magX = magY = magZ = 0.0f;
    magSensitivity = 0.15f; // AK09916: 0.15 μT/LSB
}

bool ICM20948::begin(uint8_t address)
{
    _address = address;

    Wire.begin();
    // Wire.setClock(400000); // 400kHz Fast Mode I2C

    setBank(0);
    uint8_t who_am_i_val;
    readRegister(UB0_WHO_AM_I, &who_am_i_val, 1);
    if (who_am_i_val != 0xEA)
    {
        return false;
    }

    writeRegister(UB0_PWR_MGMT_1, 0x01);
    delay(100);

    writeRegister(UB0_PWR_MGMT_2, 0x00);

    setBank(2);

    // ジャイロ設定: FSR=±250 dps, DLPF有効
    writeRegister(UB2_GYRO_CONFIG_1, 0x01);
    gyroSensitivity = 131.0f;
    // ±250 dpsの場合

    // 加速度計設定: FSR=±2g, DLPF有効
    writeRegister(UB2_ACCEL_CONFIG, 0x01);
    accelSensitivity = 16384.0f;
    // ±2gの場合

    setBank(0);

    writeRegister(UB0_INT_PIN_CFG, 0x02);

    return true;
}

void ICM20948::readSensors()
{
    setBank(0);

    uint8_t data[14];
    // 加速度計の開始アドレスから14バイトを一括読み込み
    readRegister(UB0_ACCEL_XOUT_H, data, 14);

    // 生データを16ビット整数に結合
    int16_t accelX_raw = (int16_t)((data[0] << 8) | data[1]);
    int16_t accelY_raw = (int16_t)((data[2] << 8) | data[3]);
    int16_t accelZ_raw = (int16_t)((data[4] << 8) | data[5]);

    int16_t temp_raw = (int16_t)((data[6] << 8) | data[7]);

    int16_t gyroX_raw = (int16_t)((data[8] << 8) | data[9]);
    int16_t gyroY_raw = (int16_t)((data[10] << 8) | data[11]);
    int16_t gyroZ_raw = (int16_t)((data[12] << 8) | data[13]);

    // 物理量に変換
    accelX = accelX_raw / accelSensitivity;
    accelY = accelY_raw / accelSensitivity;
    accelZ = accelZ_raw / accelSensitivity;

    gyroX = gyroX_raw / gyroSensitivity;
    gyroY = gyroY_raw / gyroSensitivity;
    gyroZ = gyroZ_raw / gyroSensitivity;

    // 温度を摂氏に変換
    temperature = (temp_raw / 333.87f) + 21.0f;
}

void ICM20948::writeRegister(uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void ICM20948::readRegister(uint8_t reg, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_address, len);

    for (uint8_t i = 0; i < len; i++)
    {
        if (Wire.available())
        {
            data[i] = Wire.read();
        }
    }
}

void ICM20948::setBank(uint8_t bank)
{
    writeRegister(REG_BANK_SEL, (bank & 0x03) << 4);
}

bool ICM20948::initMagnetometer()
{
    // Check magnetometer WHO_AM_I
    if (!checkMagnetometer())
    {
        return false;
    }

    // Set continuous measurement mode (20Hz) following Waveshare approach
    writeSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_WRITE,
                   REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_20HZ);
    delay(10);

    return true;
}

void ICM20948::readMagnetometer()
{
    uint8_t counter = 20;
    uint8_t data[MAG_DATA_LEN];
    int16_t rawBuf[3] = {0};
    int32_t avgBuf[3] = {0};
    static MagAvgData avgData[3];

    // Wait for data ready - checking ST2 register like Waveshare
    while (counter > 0)
    {
        delay(10);
        readSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ,
                      REG_ADD_MAG_ST2, 1, data);

        if ((data[0] & 0x01) != 0)
            break;

        counter--;
    }

    if (counter != 0)
    {
        // Read magnetometer data (6 bytes)
        readSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ,
                      REG_ADD_MAG_DATA, MAG_DATA_LEN, data);

        // Convert to 16-bit signed values (little endian)
        rawBuf[0] = ((int16_t)data[1] << 8) | data[0];
        rawBuf[1] = ((int16_t)data[3] << 8) | data[2];
        rawBuf[2] = ((int16_t)data[5] << 8) | data[4];
    }

    // Apply averaging like Waveshare implementation
    for (uint8_t i = 0; i < 3; i++)
    {
        calcAvgValue(&avgData[i].index, avgData[i].buffer, rawBuf[i], &avgBuf[i]);
    }

    // Convert to μT and apply Waveshare coordinate adjustments
    magX = avgBuf[0] * magSensitivity;
    magY = -avgBuf[1] * magSensitivity; // Y inverted like Waveshare
    magZ = -avgBuf[2] * magSensitivity; // Z inverted like Waveshare
}

float ICM20948::getHeading()
{
    float heading = atan2(magY, magX) * 180.0 / M_PI;

    // Normalize to 0-360 degrees
    if (heading < 0)
    {
        heading += 360.0;
    }

    return heading;
}

float ICM20948::getTiltCompensatedHeading()
{
    // Calculate roll and pitch from accelerometer
    float roll = atan2(accelY, accelZ);
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));

    // Tilt compensation
    float magX_comp = magX * cos(pitch) + magZ * sin(pitch);
    float magY_comp = magX * sin(roll) * sin(pitch) + magY * cos(roll) - magZ * sin(roll) * cos(pitch);

    // Calculate heading
    float heading = atan2(magY_comp, magX_comp) * 180.0 / M_PI;

    // Normalize to 0-360 degrees
    if (heading < 0)
    {
        heading += 360.0;
    }

    return heading;
}

float ICM20948::getMagneticFieldStrength()
{
    return sqrt(magX * magX + magY * magY + magZ * magZ);
}

bool ICM20948::needsMagnetometerCalibration()
{
    float fieldStrength = getMagneticFieldStrength();

    // Earth's magnetic field is typically 25-65 μT
    const float MIN_FIELD_STRENGTH = 15.0f;
    const float MAX_FIELD_STRENGTH = 100.0f;

    return (fieldStrength < MIN_FIELD_STRENGTH || fieldStrength > MAX_FIELD_STRENGTH);
}

// Helper methods for magnetometer I2C communication based on Waveshare implementation
bool ICM20948::checkMagnetometer()
{
    bool result = false;
    uint8_t data[2];

    readSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ,
                  REG_ADD_MAG_WIA1, 2, data);

    if ((data[0] == REG_VAL_MAG_WIA1) && (data[1] == REG_VAL_MAG_WIA2))
    {
        result = true;
    }

    return result;
}

void ICM20948::readSecondary(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t temp;

    setBank(3); // Switch to bank 3
    writeRegister(REG_ADD_I2C_SLV0_ADDR, addr);
    writeRegister(REG_ADD_I2C_SLV0_REG, reg);
    writeRegister(REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | len);

    setBank(0); // Switch to bank 0

    temp = readRegister8(UB0_USER_CTRL);
    temp |= REG_VAL_BIT_I2C_MST_EN;
    writeRegister(UB0_USER_CTRL, temp);
    delay(5);
    temp &= ~REG_VAL_BIT_I2C_MST_EN;
    writeRegister(UB0_USER_CTRL, temp);

    for (uint8_t i = 0; i < len; i++)
    {
        data[i] = readRegister8(REG_ADD_EXT_SENS_DATA_00 + i);
    }

    setBank(3); // Switch to bank 3
    temp = readRegister8(REG_ADD_I2C_SLV0_CTRL);
    temp &= ~(REG_VAL_BIT_SLV0_EN | 0x0F); // Clear enable bit and length
    writeRegister(REG_ADD_I2C_SLV0_CTRL, temp);

    setBank(0); // Switch to bank 0
}

void ICM20948::writeSecondary(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t temp;

    setBank(3); // Switch to bank 3
    writeRegister(REG_ADD_I2C_SLV1_ADDR, addr);
    writeRegister(REG_ADD_I2C_SLV1_REG, reg);
    writeRegister(REG_ADD_I2C_SLV1_DO, data);
    writeRegister(REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN | 1);

    setBank(0); // Switch to bank 0

    temp = readRegister8(UB0_USER_CTRL);
    temp |= REG_VAL_BIT_I2C_MST_EN;
    writeRegister(UB0_USER_CTRL, temp);
    delay(5);
    temp &= ~REG_VAL_BIT_I2C_MST_EN;
    writeRegister(UB0_USER_CTRL, temp);

    setBank(3); // Switch to bank 3
    temp = readRegister8(REG_ADD_I2C_SLV0_CTRL);
    temp &= ~(REG_VAL_BIT_SLV0_EN | 0x0F); // Clear enable bit and length
    writeRegister(REG_ADD_I2C_SLV0_CTRL, temp);

    setBank(0); // Switch to bank 0
}

void ICM20948::calcAvgValue(uint8_t *index, int16_t *avgBuffer, int16_t inVal, int32_t *outVal)
{
    avgBuffer[(*index)++] = inVal;
    *index &= 0x07; // Keep index in range 0-7

    *outVal = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        *outVal += avgBuffer[i];
    }
    *outVal >>= 3; // Divide by 8
}

uint8_t ICM20948::readRegister8(uint8_t reg)
{
    uint8_t data;
    readRegister(reg, &data, 1);
    return data;
}
