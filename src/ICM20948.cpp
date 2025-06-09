#include "ICM20948.h"

ICM20948::ICM20948()
{
    // コンストラクタ：変数を初期化
    accelX = accelY = accelZ = 0.0f;
    gyroX = gyroY = gyroZ = 0.0f;
    temperature = 0.0f;
    accelSensitivity = 1.0f;
    gyroSensitivity = 1.0f;
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