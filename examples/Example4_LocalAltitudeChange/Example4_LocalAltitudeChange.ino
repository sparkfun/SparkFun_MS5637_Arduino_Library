/*
  Reading barometric pressure from the MS5637
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 13th, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14688

  This example shows how to detect a local altitude change. At power up the sensor will
  take a series of readings, average them, and use that average pressure as a baseline.
  Moving up or down a flight of stairs will show a change in altitude.
*/

#include <Wire.h>

#include "SparkFun_MS5637_Arduino_Library.h"

MS5637 barometricSensor;

float startingPressure = 0.0;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Qwiic Pressure Sensor MS5637 Example");

  Wire.begin();

  if (barometricSensor.begin() == false)
  {
    Serial.println("MS5637 sensor did not respond. Please check wiring.");
  }

  //Set the resolution of the sensor to the highest level of resolution: 0.016 mbar
  barometricSensor.setResolution(ms5637_resolution_osr_8192);
  
  //Take 16 readings and average them
  startingPressure = 0.0;
  for (int x = 0 ; x < 16 ; x++)
    startingPressure += barometricSensor.getPressure();
  startingPressure /= (float)16;

  Serial.print("Starting pressure=");
  Serial.print(startingPressure);
  Serial.println("hPa");    
}

void loop(void) {

  float currentPressure = barometricSensor.getPressure();

  Serial.print("Pressure=");
  Serial.print(currentPressure, 3);
  Serial.print("(hP or mbar)");

  float altitudeDelta = barometricSensor.altitudeChange(currentPressure, startingPressure);
  Serial.print(" Change in Altitude=");
  Serial.print(altitudeDelta, 1);
  Serial.print("m");

  float altitudeFeet = altitudeDelta * 3.28084;
  Serial.print("/");
  Serial.print(altitudeFeet, 1);
  Serial.print("ft");

  Serial.println();

  delay(10);
}



