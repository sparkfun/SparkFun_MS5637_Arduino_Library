/*
  This is the original example provided by TE, the manufacturer of the MS5637.
*/

#include "ms5637.h"

static ms5637 m_ms5637;

void setup(void) {
  Serial.begin(9600);

  Serial.println("==== TE Connectivity ====");
  Serial.println("======== MS5637 =========");
  Serial.println("======== Measure =========");

  m_ms5637.begin();
  
}
void loop(void) {
  ms5637_status status;
  float temperature;
  float pressure;
  boolean connected;

  connected = m_ms5637.is_connected();
  if (connected) {
    Serial.println(connected ? "Sensor Connected" : "Sensor Disconnected");

    status = m_ms5637.read_temperature_and_pressure(&temperature, &pressure);

    Serial.print("---Temperature = ");
    Serial.print(temperature, 1);
    Serial.print((char)176); //Degree symbol
    Serial.println("C");

    Serial.print("---Pressure = ");
    Serial.print(pressure, 1);
    Serial.println("hPa");
  } else {
    Serial.println(connected ? "Sensor Connected" : "Sensor Disconnected");
  }

  delay(1000);
}
