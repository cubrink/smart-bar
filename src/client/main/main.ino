#include <SparkFunMPU9250-DMP.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               10); // Set DMP FIFO rate to 10 Hz

}

void loop() {
  // put your main code here, to run repeatedly:

}
