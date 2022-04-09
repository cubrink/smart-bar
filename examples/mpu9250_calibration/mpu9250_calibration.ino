#include "MPU9250.h"

MPU9250 mpu;

void setup()
{
    Serial.begin(115200);

    Wire.begin();

    delay(2000);
    mpu.setup();

    delay(5000);

    // calibrate if ya want to 
    Serial.println("Updating gyro and accel");
    mpu.calibrateAccelGyro();
    Serial.println("Updatating mag");
    mpu.calibrateMag();

    mpu.printCalibration();
}

void loop()
{
}
