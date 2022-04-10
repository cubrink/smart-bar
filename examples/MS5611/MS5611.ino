//
//    FILE: MS5611_test.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo application
//    DATE: 2014-okt-16
//     URL: https://github.com/RobTillaart/MS5611


#include "MS5611.h"
#include <SimpleKalmanFilter.h>
#include <cppQueue.h>



MS5611 MS5611(0x77);

#ifndef LED_BUILTIN
#define LED_BUILTIN    13
#endif

enum BAR_STATE
{
  UNDEFINED = 0,
  STATIONARY = 1,
  RISING_0 = 2,
  FALLING_0 = 3,
  STATIONARY_UP = 4,
  STATIONARY_DN = 5,
  RISING_1 = 6,
  FALLING_1 = 7,
  UNRACKED = 8,
  RACKED = 9,
  REP_FINISHED = 10
};

enum BAR_ACTION
{
  MOVING_STATIONARY = 0,
  MOVING_UP = 1,
  MOVING_DN = 2
};

enum REP_DIRECTION
{
  START_LOW = 0,        // ex. shoulder press
  START_HIGH = 1,       // ex. squat, bench press
  REP_UNKNOWN = 2
};



// return the pressure difference
float get_pressure_difference(const float int_avg, float &prev_value);

// return the current status of the bar
//  press_diff: difference in pressure
BAR_ACTION get_bar_action(const float press_diff, const float cur_avg);

// update the queue to get latest average raw pressure value
float updateQueue();






const float MOVE_UP_THRESHOLD = 20.0;     // pressure difference threshold for bar movement upward
const float MOVE_DN_THRESHOLD = -20.0;    // pressure difference threshold for bar movement downward
const int THRESHOLD_UP_MET_CNT = 10;      // number of scans required above UP threshold to satisfy bar movement upward
const int THRESHOLD_DN_MET_CNT = 10;      // number of scans required below DN threshold to satisfy bar movement downward
const int STATIONARY_MET_CNT = 50;        // number of scans required to satisfy racked state

int threshold_up_cntr = 0;                // number of scans above up threshold
int threshold_dn_cntr = 0;                // number of scans below dn threshold
int stationary_cntr = 0;                  // number of scans with no movement

BAR_STATE cur_bar_state = RACKED;
BAR_ACTION cur_bar_action = MOVING_STATIONARY;
REP_DIRECTION cur_rep_direction = REP_UNKNOWN;
unsigned long motion_timer;
unsigned long motion_hold_timer;


// THIS NEEDS TO BE SET TO 0 WHENEVER BYTE IS SENT
int rep_count = 0;  

// THIS IS FOR TESTING ONLY
int rep_accumulator = 0;


float avg_raw = 0;
float prev_pressure_value = 0.0;
float pressure_difference = 0.0;

const int PRESSURE_QUEUE_MAX_SIZE = 20;
cppQueue pressure_queue(sizeof(float), PRESSURE_QUEUE_MAX_SIZE, FIFO);
float pressure_queue_sum = 0.0;
unsigned long queue_timer;
unsigned long queue_timer_cycle_wait = 10000;
unsigned long queue_timer_cycle_scan = 500;

SimpleKalmanFilter pressureKalmanFilter(0.01, 0.01, 0.1);
SimpleKalmanFilter pressureDifferenceKalmanFilter(20.0, 20.0, 0.05);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;



void setup()
{
  Serial.begin(115200);
  while (!Serial);
  while (!MS5611.begin());


  MS5611.setOversampling(OSR_ULTRA_HIGH);

  avg_raw = set_initial_avg_pressure();
  prev_pressure_value = avg_raw;

  // Initialize queue timer
  queue_timer = millis();

  // initialize motion timers
  motion_timer = millis();
  motion_hold_timer = millis();

  //Serial.print("Initial Average: ");
  //Serial.println(avg_raw);
  
}



void loop()
{
  // zero data
  unsigned long time_passed = 0;

  // record the pressure difference
  pressure_difference = get_pressure_difference(avg_raw, prev_pressure_value);

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
  
  
  // state machine logic
  switch(cur_bar_state)
  {
    case STATIONARY:
      // save time passed since last action
      time_passed = (millis() - motion_timer);

      // if bar is moving upwards after 250 ms and rep direction should not start high, change bar state to rising
      if((cur_bar_action == MOVING_UP) && (time_passed >= 250) && (cur_rep_direction != START_HIGH))
      {
        cur_bar_state = RISING_0;
        if(cur_rep_direction == REP_UNKNOWN)
        {
          // save rep start direction - direction reset when re-racked
          cur_rep_direction = START_LOW;
        }
      }
      // if bar is moving downwards after 250 ms and rep direction should not start low, change bar state to falling
      else if((cur_bar_action == MOVING_DN) && (time_passed >= 250) && (cur_rep_direction != START_LOW))
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

      // **** REMOVE LATER ****
      // REP ACCUMULATOR IS NOT VALID IN THIS PROGRAM
      rep_accumulator = 0;
      
      break;

    case REP_FINISHED:
      // Rep has been completed and must be recorded. rep_count will be sent over bluetooth and cleared after being sent.
      // wait for rep_count to clear before starting next rep
    
      // TESTING POURPOSES ONLY
      if(rep_count == 0)
      {
        rep_accumulator++;
      }
      
      rep_count = 1;

      //TESTING PURPOSES ONLLY!!!!!
      rep_count = 0;
      
      // wait for rep_count to be set back to zero
      if(!rep_count)
      {
        cur_bar_state = STATIONARY;
      }
      break;

    default:
      Serial.println("An Error occured with the STATE MACHINE");
  }








  //Serial.print(temp_avg_raw);

  //Serial.println();
  Serial.print("\t");

  Serial.print(cur_bar_state);
  Serial.print("\t");
  Serial.print(rep_accumulator);
  Serial.println();

    

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
