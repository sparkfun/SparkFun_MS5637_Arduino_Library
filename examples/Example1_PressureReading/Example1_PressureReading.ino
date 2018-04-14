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
*/

#include <Wire.h>

#include "ms5637.h"

MS5637 barometricSensor;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Qwiic Pressure Sensor MS5637 Example");

  if(barometricSensor.begin() == false)
  {
    Serial.println("MS5637 sensor did not respond. Please check wiring.");
  }
}

void loop(void) {
  ms5637_status status;
  float temperature;
  float pressure;
  boolean connected;

  connected = barometricSensor.isConnected();
  if (connected) {
    Serial.println(connected ? "Sensor Connected" : "Sensor Disconnected");

    status = barometricSensor.read_temperature_and_pressure(&temperature, &pressure);

    Serial.print("---Temperature = ");
    Serial.print(temperature, 1);
    Serial.print((char)176);
    Serial.println("C");

    Serial.print("---Pressure = ");
    Serial.print(pressure, 1);
    Serial.println("hPa");
  } else {
    Serial.println(connected ? "Sensor Connected" : "Sensor Disconnected");
  }

  delay(1000);
}
