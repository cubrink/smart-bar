#ifndef REPTRACKER_H
#define REPTRACKER_H

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

const float MOVE_UP_THRESHOLD = 20.0;     // pressure difference threshold for bar movement upward
const float MOVE_DN_THRESHOLD = -20.0;    // pressure difference threshold for bar movement downward
const int THRESHOLD_UP_MET_CNT = 10;      // number of scans required above UP threshold to satisfy bar movement upward
const int THRESHOLD_DN_MET_CNT = 10;      // number of scans required below DN threshold to satisfy bar movement downward
const int STATIONARY_MET_CNT = 50;        // number of scans required to satisfy racked state

int threshold_up_cntr = 0;                // number of scans above up threshold
int threshold_dn_cntr = 0;                // number of scans below dn threshold
int stationary_cntr = 0;                  // number of scans with no movement


#endif 