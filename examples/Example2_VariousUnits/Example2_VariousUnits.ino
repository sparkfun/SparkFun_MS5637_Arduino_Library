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

  This example prints the current pressure and temp in various units.
*/

#include <Wire.h>

#include "SparkFun_MS5637_Arduino_Library.h"

MS5637 barometricSensor;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Qwiic Pressure Sensor MS5637 Example");

  Wire.begin();

  if (barometricSensor.begin() == false)
  {
    Serial.println("MS5637 sensor did not respond. Please check wiring.");
  }
}

void loop(void) {

  float temperature = barometricSensor.getTemperature();
  float pressure = barometricSensor.getPressure();

  Serial.print("Temp=");
  Serial.print(temperature, 1);
  Serial.print("(C)");

  float tempF = temperature * 9.0/5.0 + 32.0;
  Serial.print(" TempF=");
  Serial.print(tempF, 1);
  Serial.print("(F)");

  Serial.print(" Press=");
  Serial.print(pressure, 3);
  Serial.print("(hPa or mbar)");

  float inHg = pressure * 0.02952998016; //0 degrees C, https://en.wikipedia.org/wiki/Inch_of_mercury
  Serial.print(" Press=");
  Serial.print(inHg, 3);
  Serial.print("(inHg)");

  float atm = pressure * 0.00098692; //https://en.wikipedia.org/wiki/Atmosphere_(unit)
  Serial.print(" Press=");
  Serial.print(atm, 3);
  Serial.print("(atm)");

  Serial.println();

  delay(10);
}



