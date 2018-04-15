/*
  Reading barometric pressure from the MS5637
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 13th, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  The original library and example code was written by TEConnectivity,
  the company that made the sensor. Way to go TE! May other companies
  learn from you.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14688

  This example prints the current pressure in hPa.

  TODO:
  output various pressure units: inHg, psi, mmHg/Torr, millibar
  output pressure change in feet - take 16 readings as baseline then output variation
  OSR 8192 is missing, oddly
  a high res example with averaging 
  diffferent wire port
  400kHz

  Read the current map from here: http://weather.unisys.com/surface/sfc_con.php?image=pr&inv=0&t=cur

  Find the four letter thing for an airport near your city: "boulder airport four letter" = KWBU, KBDU

  pressure corrected to sea level calculator

  Est local alittude: 5373.6ft, 1637.9m from https://www.freemaptools.com/elevation-finder.htm
  
  I have pressure from local airport: 24.75inHg at 1625m
  I have current absolute atmospheric pressure: 835.7hPa = 24.678 inHg

  Converting between units: https://learn.sparkfun.com/tutorials/bmp180-barometric-pressure-sensor-hookup-/measuring-weather-and-altitude
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

  Wire.begin();

  if (barometricSensor.begin() == false)
  {
    Serial.println("MS5637 sensor did not respond. Please check wiring.");
  }
//  barometricSensor.setResolution(ms5637_resolution_osr_4096);
  barometricSensor.setResolution(ms5637_resolution_osr_8192);
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

  Serial.print("Time = ");
  Serial.print(millis());

  Serial.print(" Temperature = ");
  Serial.print(temperature, 1);
  Serial.print("C");

  Serial.print(" Pressure = ");
  Serial.print(avgPressure, 3);
  Serial.print("hPa (mbar)");

  float inHg = avgPressure * 0.029529980164712;
  Serial.print(" Pressure = ");
  Serial.print(inHg, 3);
  Serial.print("inHg");

  float adjustedSeaLevel = barometricSensor.adjustToSeaLevel(avgPressure, 1637.9); //Est local alittude: 5373.6ft, 1637.9m from https://www.freemaptools.com/elevation-finder.htm
  Serial.print(" adjustedSeaLevel pressure = ");
  Serial.print(adjustedSeaLevel, 3);
  Serial.print("hPa");

  inHg = adjustedSeaLevel * 0.029529980164712;
  Serial.print(" Pressure = ");
  Serial.print(inHg, 3);
  Serial.print("inHg");

  Serial.println();

  delay(10);
}



