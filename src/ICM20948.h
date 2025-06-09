#ifndef ICM20948_H
#define ICM20948_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class ICM20948
{
private:
    uint8_t _address;

    // Sensor data
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ; // Magnetometer data
    float temperature;

    // Sensitivity values
    float accelSensitivity;
    float gyroSensitivity;
    float magSensitivity; // Magnetometer sensitivity

    // Register addresses - Bank 0
    static const uint8_t REG_BANK_SEL = 0x7F;
    static const uint8_t UB0_WHO_AM_I = 0x00;
    static const uint8_t UB0_PWR_MGMT_1 = 0x06;
    static const uint8_t UB0_PWR_MGMT_2 = 0x07;
    static const uint8_t UB0_INT_PIN_CFG = 0x0F;
    static const uint8_t UB0_USER_CTRL = 0x03;
    static const uint8_t UB0_ACCEL_XOUT_H = 0x2D;

    // Register addresses - Bank 2
    static const uint8_t UB2_GYRO_CONFIG_1 = 0x01;
    static const uint8_t UB2_ACCEL_CONFIG = 0x14;

    // AK09916 magnetometer registers
    static const uint8_t AK09916_I2C_ADDR = 0x0C;
    static const uint8_t AK09916_WIA1 = 0x00;
    static const uint8_t AK09916_WIA2 = 0x01;
    static const uint8_t AK09916_ST1 = 0x10;
    static const uint8_t AK09916_HXL = 0x11;
    static const uint8_t AK09916_ST2 = 0x18;
    static const uint8_t AK09916_CNTL2 = 0x31;
    static const uint8_t AK09916_CNTL3 = 0x32;

    // Magnetometer constants from Waveshare
    static const uint8_t I2C_ADD_ICM20948_AK09916 = 0x0C;
    static const uint8_t I2C_ADD_ICM20948_AK09916_READ = 0x80;
    static const uint8_t I2C_ADD_ICM20948_AK09916_WRITE = 0x00;
    static const uint8_t REG_ADD_MAG_WIA1 = 0x00;
    static const uint8_t REG_VAL_MAG_WIA1 = 0x48;
    static const uint8_t REG_ADD_MAG_WIA2 = 0x01;
    static const uint8_t REG_VAL_MAG_WIA2 = 0x09;
    static const uint8_t REG_ADD_MAG_ST2 = 0x10;
    static const uint8_t REG_ADD_MAG_DATA = 0x11;
    static const uint8_t REG_ADD_MAG_CNTL2 = 0x31;
    static const uint8_t REG_VAL_MAG_MODE_20HZ = 0x04;
    static const uint8_t MAG_DATA_LEN = 6;

    // User bank 3 register addresses
    static const uint8_t REG_ADD_I2C_SLV0_ADDR = 0x03;
    static const uint8_t REG_ADD_I2C_SLV0_REG = 0x04;
    static const uint8_t REG_ADD_I2C_SLV0_CTRL = 0x05;
    static const uint8_t REG_VAL_BIT_SLV0_EN = 0x80;
    static const uint8_t REG_ADD_I2C_SLV1_ADDR = 0x07;
    static const uint8_t REG_ADD_I2C_SLV1_REG = 0x08;
    static const uint8_t REG_ADD_I2C_SLV1_CTRL = 0x09;
    static const uint8_t REG_ADD_I2C_SLV1_DO = 0x0A;
    static const uint8_t REG_ADD_EXT_SENS_DATA_00 = 0x3B;
    static const uint8_t REG_VAL_BIT_I2C_MST_EN = 0x20;

    // Averaging data structure
    struct MagAvgData
    {
        uint8_t index;
        int16_t buffer[8];
    };

    // Static averaging buffers for magnetometer
    static MagAvgData magAvgBuf[3]; // Helper methods
    void writeRegister(uint8_t reg, uint8_t data);
    void readRegister(uint8_t reg, uint8_t *data, uint8_t len);
    uint8_t readRegister8(uint8_t reg);
    void setBank(uint8_t bank);

    // Magnetometer helper methods
    bool checkMagnetometer();
    void readSecondary(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
    void writeSecondary(uint8_t addr, uint8_t reg, uint8_t data);
    void calcAvgValue(uint8_t *index, int16_t *avgBuffer, int16_t inVal, int32_t *outVal);

public:
    ICM20948();

    // Initialization
    bool begin(uint8_t address = 0x68);
    bool initMagnetometer();

    // Data reading
    void readSensors();
    void readMagnetometer();

    // Accelerometer getters
    float getAccelX() const { return accelX; }
    float getAccelY() const { return accelY; }
    float getAccelZ() const { return accelZ; }

    // Gyroscope getters
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }

    // Magnetometer getters
    float getMagX() const { return magX; }
    float getMagY() const { return magY; }
    float getMagZ() const { return magZ; }

    // Magnetometer calculations
    float getHeading();
    float getTiltCompensatedHeading();
    float getMagneticFieldStrength();
    bool needsMagnetometerCalibration();

    // Temperature getter
    float getTemperature() const { return temperature; }
};

#endif