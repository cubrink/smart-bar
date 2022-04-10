#include <SparkFunMPU9250-DMP.h>
#include <Fusion.h>
#include <SimpleKalmanFilter.h>

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



/* --- Begin configuration for Kalman Filters  --- */
// Acceleration
SimpleKalmanFilter KF_ax(0.06, 0.06, 0.1);
SimpleKalmanFilter KF_ay(0.06, 0.06, 0.1);
SimpleKalmanFilter KF_az(0.08, 0.08, 0.1);
// Velocity
SimpleKalmanFilter KF_vx(20*0.12, 20*0.12, 0.1);
SimpleKalmanFilter KF_vy(20*0.12, 20*0.12, 0.1);
SimpleKalmanFilter KF_vz(20*0.16, 20*0.16, 0.1);
// Position
SimpleKalmanFilter KF_px(0.24, 0.24, 0.1);
SimpleKalmanFilter KF_py(0.24, 0.24, 0.1);
SimpleKalmanFilter KF_pz(0.32, 0.32, 0.1);

float kf_ax = 0;
float kf_ay = 0;
float kf_az = 0;

float kf_vx = 0;
float kf_vy = 0;
float kf_vz = 0;

float kf_px = 0;
float kf_py = 0;
float kf_pz = 0;

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
  
  previousTimestamp = millis();
}

void loop() {
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

  float deltaTime = (float) (timestamp - previousTimestamp) / 1000.0f;
  previousTimestamp = timestamp;

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

  // Ignore acceleration from gravity
  FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
  
//  printf("%0.1f,%0.1f,%0.1f,", earth.axis.x, earth.axis.y, earth.axis.z);
//  printf("%0.1f,%0.1f,%0.1f\n", imu.roll, imu.pitch, imu.yaw); 

  // Update acceleration, velocity and position tracking
  float kf_ax_prev = kf_ax;
  float kf_ay_prev = kf_ay;
  float kf_az_prev = kf_az;

  float kf_vx_prev = kf_vx;
  float kf_vy_prev = kf_vy;
  float kf_vz_prev = kf_vz;
 
  kf_ax = KF_ax.updateEstimate(earth.axis.x);
  kf_ay = KF_ay.updateEstimate(earth.axis.y);
  kf_az = KF_az.updateEstimate(earth.axis.z);

  kf_vx = KF_vx.updateEstimate((20*kf_ax - 20*kf_ax_prev)/deltaTime);
  kf_vy = KF_vy.updateEstimate((20*kf_ay - 20*kf_ay_prev)/deltaTime);
  kf_vz = KF_vz.updateEstimate((20*kf_az - 20*kf_az_prev)/deltaTime);

  kf_px = KF_px.updateEstimate((kf_vx - kf_vx_prev)/deltaTime);
  kf_py = KF_py.updateEstimate((kf_vy - kf_vy_prev)/deltaTime);
  kf_pz = KF_pz.updateEstimate((kf_vz - kf_vy_prev)/deltaTime);

//  printf("%0.1f,%0.1f,%0.1f\n", kf_vx*5, kf_vy*5, kf_vz*5);
  printf("%0.1f\n", 5*kf_vy);

//  printf("%0.1f,%0.1f,%0.1f\n", kf_ax*5, kf_ay*5, kf_az*5);
}
