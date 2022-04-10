#include <SparkFunMPU9250-DMP.h>
#include <Fusion.h>
#include <SimpleKalmanFilter.h>
#include "BluetoothSerial.h"
#include "RepTracker.h"

#include <stdio.h>
#include <cppQueue.h>


#ifdef defined(SAMD)
 #define SerialPort SerialUSB
#else
  #define SerialPort Serial
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN    13
#endif


// update the queue to get latest average raw pressure value
float updateQueue();

float get_pressure_difference(const float int_avg, float &prev_value);
float set_initial_avg_pressure();

// Buzzer attached to GIPO pin 33
const int BUZZER = 33;
unsigned long buzzer_timer;

const unsigned int SAMPLE_RATE = 100;
unsigned long previousTimestamp;

MPU9250_DMP imu;
BluetoothSerial SerialBT;
MS5611 MS5611(0x77);



/* --- Begin configuration for Kalman Filters  --- */
// Acceleration
SimpleKalmanFilter KF_a(0.20, 0.20, 1);
SimpleKalmanFilter KF_v(.1,.1,1);

float kf_a = 0;
float kf_v = 0;

// Pressure
SimpleKalmanFilter pressureKalmanFilter(0.01, 0.01, 0.1);
SimpleKalmanFilter pressureDifferenceKalmanFilter(20.0, 20.0, 0.05);

/* --- End configuration for Kalman Filters --- */

/* --- Begin Rep Counter Config --- */
int rep_count = 0;  
BAR_STATE cur_bar_state = RACKED;
BAR_ACTION cur_bar_action = MOVING_STATIONARY;
REP_DIRECTION cur_rep_direction = REP_UNKNOWN;
unsigned long motion_timer;
unsigned long motion_hold_timer;

float avg_raw = 0;
float prev_pressure_value = 0.0;
float pressure_difference = 0.0;

const int PRESSURE_QUEUE_MAX_SIZE = 20;
cppQueue pressure_queue(sizeof(float), PRESSURE_QUEUE_MAX_SIZE, FIFO);
float pressure_queue_sum = 0.0;
unsigned long queue_timer;
unsigned long queue_timer_cycle_wait = 10000;
unsigned long queue_timer_cycle_scan = 500;

/* --- End Rep Counter Config --- */

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
   SerialPort.begin(115200);

  while (!Serial);
  while (!MS5611.begin());

  // Initialize GIOP
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  // Set high resolution on barometric pressure
  MS5611.setOversampling(OSR_ULTRA_HIGH);
  avg_raw = set_initial_avg_pressure();
  prev_pressure_value = avg_raw;

  // Initialize queue timer
  queue_timer = millis();

  // initialize motion timers
  motion_timer = millis();
  motion_hold_timer = millis();


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

  // Various MPU9250 configurations
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT, 10); // Set DMP FIFO rate to 10 Hz, Enable 6-axis quat
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(5); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz

  // Initialize AHRS algorithm
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);
  FusionAhrsSetSettings(&ahrs, &settings);

  // Turn on Bluetooth discovery
  SerialBT.begin("ESP32test"); 
  
  previousTimestamp = millis();
}

void loop() {

  ///* ---------------- Begin IMU Processing ---------------- *///
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

  /* --- End Read IMU --- */

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

  // Update acceleration, velocity and position tracking
  float kf_a_prev = kf_a;
  float kf_v_prev = kf_v;
 
  kf_a = KF_a.updateEstimate(sqrt(earth.axis.x*earth.axis.x + earth.axis.y*earth.axis.y + earth.axis.z*earth.axis.z));
  kf_a = max(0.0f, kf_a);
  kf_v = KF_v.updateEstimate(kf_v + deltaTime*kf_a);
  kf_v = max(0.0f, kf_v);


  ///* ---------------- End IMU Processing ---------------- *///

  ///* ---------------- Begin Rep Tracking  ---------------- *///


  // zero data
  unsigned long time_passed = 0;

  // record the pressure difference
  pressure_difference = get_pressure_difference(avg_raw, prev_pressure_value);


  //
  // Update running average of pressure difference info
  //

  // if bar is racked, zero out the height data every once in a while
  // this does not occur all the time to avoid smoothing the pressure difference signal
  if(cur_bar_state == RACKED && (millis() - queue_timer < queue_timer_cycle_scan))
  {
    avg_raw = updateQueue();
  }
  else if(millis() - queue_timer >= queue_timer_cycle_wait)
  {
    queue_timer = millis();
  }

  //
  // Accumulate values if running average meets threshold
  //
  
  // update threshold counts
  if(pressure_difference >= MOVE_UP_THRESHOLD)
  {
    // bar is moving up
    threshold_up_cntr += 1;
    threshold_dn_cntr = 0;
    stationary_cntr = 0;
  }
  else if(pressure_difference <= MOVE_DN_THRESHOLD)
  {
    // bar is moving down
    threshold_dn_cntr += 1;
    threshold_up_cntr = 0;
    stationary_cntr = 0;
  }
  else
  {
    // bar neither moving up or down
    stationary_cntr += 1;
    threshold_up_cntr = 0;
    threshold_dn_cntr = 0;
  }


  //
  // Determine motion type of bar
  //

  // determine current action from threshold counts
  if(cur_bar_action != MOVING_UP && threshold_up_cntr >= THRESHOLD_UP_MET_CNT)
  {
    cur_bar_action = MOVING_UP;
    motion_timer = millis();
  }
  else if(cur_bar_action != MOVING_DN && threshold_dn_cntr >= THRESHOLD_DN_MET_CNT)
  {
    cur_bar_action = MOVING_DN;
    motion_timer = millis();
  }
  else if(cur_bar_action != MOVING_STATIONARY && stationary_cntr >= STATIONARY_MET_CNT)
  {
    cur_bar_action = MOVING_STATIONARY;
    motion_timer = millis();
  }
  
  
  //
  // Move between states using state machine 
  //

  switch(cur_bar_state)
  {
    case STATIONARY:
      // save time passed since last action
      time_passed = (millis() - motion_timer);

      // if bar is moving upwards after 500 ms and rep direction should not start high, change bar state to rising
      if((cur_bar_action == MOVING_UP) && (time_passed >= 500) && (cur_rep_direction != START_HIGH))
      {
        cur_bar_state = RISING_0;
        if(cur_rep_direction == REP_UNKNOWN)
        {
          // save rep start direction - direction reset when re-racked
          cur_rep_direction = START_LOW;
        }
      }
      // if bar is moving downwards after 500 ms and rep direction should not start low, change bar state to falling
      else if((cur_bar_action == MOVING_DN) && (time_passed >= 500) && (cur_rep_direction != START_LOW))
      {
        cur_bar_state = FALLING_0;
        if(cur_rep_direction == REP_UNKNOWN)
        {
          // save rep start direciton - direction reset when re-racked
          cur_rep_direction = START_HIGH;
        }
      }
      // if bar is not moving for 20 seconds, change bar state to racked
      else if((cur_bar_action == MOVING_STATIONARY) && (time_passed >= 20000))
      {
        cur_bar_state = RACKED;
      }
      break;

    case RISING_0:
      // bar is now rising after previously not moving
      // if bar stops rising, change bar state to not moving (but in the middle of a move)
      if(cur_bar_action == MOVING_STATIONARY)
      {
        cur_bar_state = STATIONARY_UP;
      }
      // if bar starts moving down, change bar state to falling
      else if(cur_bar_action == MOVING_DN)
      {
        cur_bar_state = FALLING_1;
      }
      break;

    case FALLING_0:
      // bar is now falling after previously not moving
      // if bar stops falling, change bar state to not moving (but in the middle of a move)
      if(cur_bar_action == MOVING_STATIONARY)
      {
        cur_bar_state = STATIONARY_DN;
      }
      // if bar starts moving up, change bar state to rising
      if(cur_bar_action == MOVING_UP)
      {
        cur_bar_state = RISING_1;
      }
      break;

    case STATIONARY_UP:
      // bar is in the middle of a up/down rep and is stationary. waiting for falling signal
      if(cur_bar_action == MOVING_DN)
      {
        cur_bar_state = FALLING_1;
      }
      break;

    case STATIONARY_DN:
      // bar is in the middle of a down/up rap and is stationary. waiting for rising signal
      if(cur_bar_action == MOVING_UP)
      {
        cur_bar_state = RISING_1;
      }
      break;

    case RISING_1:
      // bar is rising during the second half of the rep. bar must be stationary or be moving back downwards to record completed rep
      if(cur_bar_action == MOVING_DN || cur_bar_action == MOVING_STATIONARY)
      {
        cur_bar_state = REP_FINISHED;
      }
      break;

    case FALLING_1:
      // bar is falling during the second half of the rep. bar must be stationary or be moving back upwards to record completed rep
      if(cur_bar_action == MOVING_UP || cur_bar_action == MOVING_STATIONARY)
      {
        cur_bar_state = REP_FINISHED;
      }
      break;

    case UNRACKED:
      // bar has been taken off the rack. bar must remain stationary for 2000 ms until a rep can start being recorded
      if((cur_bar_action == MOVING_STATIONARY) && ((millis() - motion_timer) >= 2000))
      {
        cur_bar_state = STATIONARY;
      }
      break;

    case RACKED:
      // bar has been completly still for a long time. set is complete or not started.
      // wait for movement to signal bar has been picked up
      cur_rep_direction = REP_UNKNOWN;
      // if movement detected (user moving bar off rack)
      if(cur_bar_action == MOVING_UP || cur_bar_action == MOVING_DN)
      {
        cur_bar_state = UNRACKED;
      }
      
      break;

    case REP_FINISHED:
      // Rep has been completed and must be recorded. rep_count will be sent over bluetooth and cleared after being sent.
      // wait for rep_count to clear before starting next rep
      rep_count = 1;

      digitalWrite(BUZZER, HIGH);
      buzzer_timer = millis();
      
      // wait for rep_count to be set back to zero
      if(!rep_count)
      {
        cur_bar_state = STATIONARY;
      }
      break;

    default:
      Serial.println("An Error occured with the STATE MACHINE");
  }

  // turn off buzzer after 1 second if on
  if(digitalRead(BUZZER) && (millis() - buzzer_timer >= 1000))
  {
    digitalWrite(BUZZER, LOW);
  }

  ///* ------------ End Rep Tracking ------------ */// 


  /* ---- Begin Bluetooth communication ---- */

  char message[1024];
  message = sprintf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%i", 
    imu.roll, imu.pitch, imu.yaw, earth.axis.x, earth.axis.y, earth.axis.z, kf_a, rep_count
  );
  SerialBT.println(message); 
  rep_count = 0;

  /* ---- End Bluetooth communication ---- */
}

float set_initial_avg_pressure()
{
  int sample_cnt = 200;
  float initial_avg = 0;
  float cur_read;
  
  for(int i = 0; i < sample_cnt; i++)
  {
    MS5611.read();
    cur_read = MS5611.getPressure();
    initial_avg += cur_read;
  }
  initial_avg = initial_avg / float(sample_cnt);
  return initial_avg;
}


float get_pressure_difference(const float int_avg, float &prev_value)
{
  bool LOCAL_DEBUG = true;
  
  // read the sensor using MS5611.read()
  // return the sensor value with MS5611.getPressure()
  MS5611.read();
  float cur_pressure_value = (pressureKalmanFilter.updateEstimate(MS5611.getPressure()) - int_avg) * -1000.0;
  if(LOCAL_DEBUG)
    Serial.print(cur_pressure_value);

  float pressure_difference = pressureDifferenceKalmanFilter.updateEstimate((cur_pressure_value - prev_value) * 20.0);
  prev_value = cur_pressure_value;
  if(LOCAL_DEBUG)
  {
    Serial.print("\t");
    Serial.print(pressure_difference);
  }
  
  return pressure_difference;
}


float updateQueue()
{
  MS5611.read();
  float pressure_raw_value = MS5611.getPressure();
  
  if(pressure_queue.isEmpty())
  {
    pressure_queue_sum = 0.0;
  }
  if(pressure_queue.isFull())
  {
    float popped_value;
    pressure_queue.pull(&popped_value);
    pressure_queue_sum -= popped_value;
  }

  pressure_queue_sum += pressure_raw_value;
  pressure_queue.push(&pressure_raw_value);

  return pressure_queue_sum / float(pressure_queue.getCount());
}
