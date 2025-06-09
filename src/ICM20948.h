#ifndef ICM20948_H
#define ICM20948_H

#include <Arduino.h>
#include <Wire.h>

class ICM20948
{
public:
    ICM20948();                         // コンストラクタ
    bool begin(uint8_t address = 0x68); // 初期化
    void readSensors();                 // センサーデータの一括読み取り

    // 物理量に変換された値を取得する関数
    float getAccelX() const { return accelX; }
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }
    float getTemperature() const { return temperature; }

private:
    // I2Cアドレス
    uint8_t _address;

    // 物理量に変換された値を格納する変数
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temperature;

    // 選択したフルスケールレンジに応じた感度
    float accelSensitivity;
    float gyroSensitivity;

    // I2C通信ヘルパー関数
    void writeRegister(uint8_t reg, uint8_t data);
    void readRegister(uint8_t reg, uint8_t *data, uint8_t len);
    void setBank(uint8_t bank);

    // レジスタアドレス
    enum Register
    {
        // Bank 0
        REG_BANK_SEL = 0x7F,
        UB0_WHO_AM_I = 0x00,
        UB0_USER_CTRL = 0x03,
        UB0_PWR_MGMT_1 = 0x06,
        UB0_PWR_MGMT_2 = 0x07,
        UB0_INT_PIN_CFG = 0x0F,
        UB0_ACCEL_XOUT_H = 0x2D,
        // Bank 2
        UB2_GYRO_CONFIG_1 = 0x01,
        UB2_ACCEL_CONFIG = 0x14
    };
};

#endif // ICM20948_H