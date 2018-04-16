/*
  Reading barometric pressure from the MS5637
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 13th, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14688

  This example shows:
    How to change the measurement resolution
    How to use a different wire port
    How to communicate at 400kHz I2C
    Take rolling average across 8 readings
*/

#include <Wire.h>

#include "SparkFun_MS5637_Arduino_Library.h"

MS5637 barometricSensor;

//Store distance readings to get rolling average
#define HISTORY_SIZE 8
float history[HISTORY_SIZE];
byte historySpot;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Qwiic Pressure Sensor MS5637 Example");

  //If your platform has multiple Wire ports you can pass the port to
  //the library and the sensor will communicate on it
  //Wire1.begin();
  //Wire1.setClock(400000); //Communicate at faster 400kHz I2C
  //barometricSensor.begin(Wire1);
  
  //Default is Wire and is optional when you call .begin()
  Wire.begin();
  Wire.setClock(400000); //Communicate at faster 400kHz I2C
  barometricSensor.begin(Wire);

  //The sensor has 6 resolution levels. The higher the resolution the longer each
  //reading takes to complete.
//  barometricSensor.setResolution(ms5637_resolution_osr_256); //1ms per reading, 0.11mbar resolution
//  barometricSensor.setResolution(ms5637_resolution_osr_512); //2ms per reading, 0.062mbar resolution
//  barometricSensor.setResolution(ms5637_resolution_osr_1024); //3ms per reading, 0.039mbar resolution
//  barometricSensor.setResolution(ms5637_resolution_osr_2048); //5ms per reading, 0.028mbar resolution
//  barometricSensor.setResolution(ms5637_resolution_osr_4096); //9ms per reading, 0.021mbar resolution
  barometricSensor.setResolution(ms5637_resolution_osr_8192); //17ms per reading, 0.016mbar resolution
}

void loop(void) {

  float temperature = barometricSensor.getTemperature();
  float pressure = barometricSensor.getPressure();

  history[historySpot] = pressure;
  if (historySpot++ == HISTORY_SIZE) historySpot = 0;

  float avgPressure = 0.0;
  for (int x = 0 ; x < HISTORY_SIZE ; x++)
    avgPressure += history[x];
  avgPressure /= (float)HISTORY_SIZE;

  Serial.print("Temperature=");
  Serial.print(temperature, 1);
  Serial.print("(C)");

  Serial.print(" Pressure=");
  Serial.print(avgPressure, 3);
  Serial.print("(hPa or mbar)");

  Serial.println();

  delay(10);
}

