//
//    FILE: MS5611_test.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo application
//    DATE: 2014-okt-16
//     URL: https://github.com/RobTillaart/MS5611


#include "MS5611.h"
#include <SimpleKalmanFilter.h>


//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77

MS5611 MS5611(0x77);

#ifndef LED_BUILTIN
#define LED_BUILTIN    13
#endif

uint32_t start, stop;



float avg = 0;

int ht_buffer_cntr = 0;
//const int BUF_MAX_CNT = 25;
//float ht_buffer[BUF_MAX_CNT];

float cur_pressure_value = 0.0;
float last_pressure_value = 0.0;
float pressure_difference = 0.0;
SimpleKalmanFilter pressureKalmanFilter(0.01, 0.01, 0.1);
SimpleKalmanFilter pressureDifferenceKalmanFilter(20.0, 20.0, 0.05);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;



void setup()
{
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);

//  Serial.println();
//  Serial.println(__FILE__);
//  Serial.print("MS5611_LIB_VERSION: ");
//  Serial.println(MS5611_LIB_VERSION);

  while (!MS5611.begin())
  {
  }



  MS5611.setOversampling(OSR_ULTRA_HIGH);

  int sample_cnt = 200;
  avg = 0;
  for(int i = 0; i < sample_cnt; i++)
  {
    MS5611.read();
    avg += MS5611.getPressure();
  }
  avg = avg / float(sample_cnt);
  last_pressure_value = avg;
  
}



void loop()
{


  // read the sensor using MS5611.read()
  // return the sensor value with MS5611.getPressure()
  MS5611.read();
  cur_pressure_value = (pressureKalmanFilter.updateEstimate(MS5611.getPressure()) - avg) * -1000.0;
  Serial.print(cur_pressure_value);

  pressure_difference = pressureDifferenceKalmanFilter.updateEstimate((cur_pressure_value - last_pressure_value) * 20.0);
  last_pressure_value = cur_pressure_value;
  Serial.print("\t");
  Serial.println(pressure_difference);

}
