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

    if (mySensor.initMagnetometer())
    {
        Serial.println("Magnetometer initialization successful.");
    }
    else
    {
        Serial.println("Magnetometer initialization failed. Stopping.");
        while (1)
        {
            Serial.println("Magnetometer initialization failed. Stopping.");
            delay(500); // Wait indefinitely
        }
    }
}

void loop()
{
    // Read sensor data and update internal variables
    mySensor.readSensors();
    mySensor.readMagnetometer(); // Add this line

    // Get and display data
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

    // 磁力計データ (μT)
    Serial.print("Magnetometer (μT): ");
    Serial.print("X=");
    Serial.print(mySensor.getMagX(), 3);
    Serial.print(" | Y=");
    Serial.print(mySensor.getMagY(), 3);
    Serial.print(" | Z=");
    Serial.println(mySensor.getMagZ(), 3);

    // ヘディング
    Serial.print("Heading (degrees): ");
    Serial.println(mySensor.getHeading(), 2);

    Serial.print("Tilt-compensated heading (degrees): ");
    Serial.println(mySensor.getTiltCompensatedHeading(), 2);

    Serial.print("Magnetic field strength (μT): ");
    Serial.println(mySensor.getMagneticFieldStrength(), 2);

    if (mySensor.needsMagnetometerCalibration())
    {
        Serial.println("Warning: Magnetometer calibration recommended!");
    }

    // 温度 (°C)
    Serial.print("Temperature (°C): ");
    Serial.println(mySensor.getTemperature(), 2);
    Serial.println();

    delay(500);
}