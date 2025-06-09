#include <Arduino.h>
#include <ICM20948.h>
#include <Wire.h>

/*
void scanI2C()
{
    Serial.println("Scanning I2C devices...");
    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address: 0x");
            Serial.println(address, HEX);
        }
    }
    Serial.println("Scan complete");
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000); // Set to 100kHz
}

void loop()
{
    scanI2C();
    delay(500);
}
*/

// Create an instance of the ICM20948 class
ICM20948 mySensor;

void scanI2C()
{
    Serial.println("Scanning I2C devices...");
    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address: 0x");
            Serial.println(address, HEX);
        }
    }
    Serial.println("Scan complete");
}

void setup()
{
    Serial.begin(115200);

    // Initialize I2C with default pins (SDA=21, SCL=22 for ESP32)
    Wire.begin();
    Wire.setClock(100000); // Set to 100kHz

    // Scan for I2C devices first
    scanI2C();

    if (mySensor.begin())
    {
        Serial.println("ICM-20948 initialization successful.");
    }
    else
    {
        Serial.println("ICM-20948 initialization failed. Stopping.");
        while (1)
        {
            Serial.println("ICM-20948 initialization failed. Stopping.");
            delay(500); // Wait indefinitely
        }
    }
}

void loop()
{
    /*
    scanI2C();
    delay(500);
    */

    // センサーデータを読み込み、内部の変数を更新
    mySensor.readSensors();

    // データを取得して表示
    Serial.println("--- Sensor Data ---");

    // 加速度 (g)
    Serial.print("Accel (g): ");
    Serial.print("X=");
    Serial.print(mySensor.getAccelX(), 3);
    Serial.print(" | Y=");
    Serial.print(mySensor.getAccelY(), 3);
    Serial.print(" | Z=");
    Serial.println(mySensor.getAccelZ(), 3);

    // ジャイロ (dps)
    Serial.print("Gyro (dps): ");
    Serial.print("X=");
    Serial.print(mySensor.getGyroX(), 3);
    Serial.print(" | Y=");
    Serial.print(mySensor.getGyroY(), 3);
    Serial.print(" | Z=");
    Serial.println(mySensor.getGyroZ(), 3);

    // 温度 (°C)
    Serial.print("Temperature (°C): ");
    Serial.println(mySensor.getTemperature(), 2);
    Serial.println();

    delay(500);
}