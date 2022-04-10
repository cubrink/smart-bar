#include <SparkFunMPU9250-DMP.h>
#include <Fusion.h>
#include <SimpleKalmanFilter.h>
#include "BluetoothSerial.h"

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

BluetoothSerial SerialBT;

/* --- Begin configuration for Kalman Filters  --- */
// Acceleration
SimpleKalmanFilter KF_a(0.20, 0.20, 1);
SimpleKalmanFilter KF_v(.1,.1,1);

float kf_a = 0;
float kf_v = 0;


/* --- End configuration for Kalman Filters --- */


/* --- Begin configuration for attitude and heading reference system (AHSR)  --- */
// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

// Calibration settings
// Acquired with blood, sweat and tears
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

/* --- End configuration for attitude and heading reference system (AHSR)  --- */


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

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT, 10); // Set DMP FIFO rate to 10 Hz, Enable 6-axis quat
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(5); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz

  
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);
  FusionAhrsSetSettings(&ahrs, &settings);

  SerialBT.begin("ESP32test"); 
  
  previousTimestamp = millis();
}

void loop() {

  /* --- Begin Read IMU --- */
  if (!imu.fifoAvailable()) {
    return;
  }
  if (imu.dmpUpdateFifo() != INV_SUCCESS) {
    return;
  }

  imu.update();
  imu.computeEulerAngles();
  unsigned long timestamp = imu.time;

  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);

  /* --- Begin Read IMU --- */
  /* --- Begin Correct IMU Data --- */
  // Acquire latest sensor data
  FusionVector gyroscope = {gyroX, gyroY, gyroZ};
  FusionVector accelerometer = {accelX, accelY, accelZ};
  FusionVector magnetometer = {magX, magY, magZ};

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
  
  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);
  
  /* --- End Correct IMU Data--- */

  /* ---  Begin AHRS Algorithm for heading and acc --- */
  float deltaTime = (float) (timestamp - previousTimestamp) / 1000.0f;
  previousTimestamp = timestamp;

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
  
  // Ignore acceleration from gravity
  FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
  /* ---  End AHRS Algorithm for heading and acc --- */

  
  
//  printf("%0.1f,%0.1f,%0.1f,", earth.axis.x, earth.axis.y, earth.axis.z);
//  printf("%0.1f,%0.1f,%0.1f\n", imu.roll, imu.pitch, imu.yaw); 

  // Update acceleration, velocity and position tracking
  float kf_a_prev = kf_a;
  float kf_v_prev = kf_v;
 
  kf_a = KF_a.updateEstimate(sqrt(earth.axis.x*earth.axis.x + earth.axis.y*earth.axis.y + earth.axis.z*earth.axis.z));
  kf_a = max(0.0f, kf_a);
  kf_v = KF_v.updateEstimate(kf_v + deltaTime*kf_a);
  kf_v = max(0.0f, kf_v);

  char message[1024];
  message = sprintf("%0.1f\n\0", kf_a);
  SerialBT.println(message); 
}
