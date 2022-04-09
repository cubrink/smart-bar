#include <SparkFunMPU9250-DMP.h>
#include <Fusion.h>

#include <stdio.h>

#ifdef defined(SAMD)
 #define SerialPort SerialUSB
#else
  #define SerialPort Serial
#endif

// replace this with actual sample rate
const unsigned int SAMPLE_RATE = 100;

MPU9250_DMP imu;
unsigned long previousTimestamp;

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;



const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {1.79f, -0.17f, -1.57f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.04f, -0.104f};
const FusionMatrix softIronMatrix = {1.524f, -0.101f, 0.247f, -0.182f, 1.308f, -0.398f, 0.155f, 0.316f, 1.886f};
const FusionVector hardIronOffset = {-65.606f, -23.791f, 17.621f};

// Set AHRS algorithm settings
const FusionAhrsSettings settings = {
        .gain = 0.5f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 5 * SAMPLE_RATE /* 5 seconds */
};


void setup() {
  // put your setup code here, to run once:
    SerialPort.begin(9600);

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  // Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(10); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);
  FusionAhrsSetSettings(&ahrs, &settings);
  
  previousTimestamp = millis();
}

void loop() {
  
  unsigned long timestamp = millis();
  imu.update();
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);

  // Acquire latest sensor data
  FusionVector gyroscope = {gyroX, gyroY, gyroZ};
  FusionVector accelerometer = {accelX, accelY, accelZ};
  FusionVector magnetometer = {magX, magY, magZ};

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
//
//  printf("%f,%f,%f\n", 
//  magnetometer.array[0], 
//  magnetometer.array[1], 
//  magnetometer.array[2]);
//  Serial.flush(); 
//  Serial.print(magX); 
//  Serial.print(",");
//  Serial.print(magY);
//  Serial.print(",");
//  Serial.print(magZ);
//  Serial.println();
  
  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);


  float deltaTime = (float) (timestamp - previousTimestamp) / 1000.0f;
  previousTimestamp = timestamp;

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

  // Print algorithm outputs
  FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

//  printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
//           euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
//           earth.axis.x, earth.axis.y, earth.axis.z);
//  printf("%0.1f,%0.1f,%0.1f,%0.1f,%0.1f,%0.1f\n",
//           euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
//           accelX, accelY, accelZ);
//  printf("%0.1f,%0.1f,%0.1f\n", earth.axis.x, earth.axis.y, earth.axis.z);
  printf("%0.1f,%0.1f,%0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
//  printf("%0.1f,%0.1f,%0.1f\n", accelerometer.array[0], accelerometer.array[1], accelerometer.array[2]);
//    printf("%0.1f,%0.1f,%0.1f\n", gyroscope.array[0], gyroscope.array[1], gyroscope.array[2]);
}
