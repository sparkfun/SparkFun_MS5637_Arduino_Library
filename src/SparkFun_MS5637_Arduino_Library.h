/*
  This is a library written for the MS5637. Originally written by TEConnectivity
  with an MIT license. Library updated and brought to fit Arduino Library standards
  by Nathan Seidle @ SparkFun Electronics, April 13th, 2018

  MEMs based barometric pressure sensors are quite common these days. The
  MS5637 shines by being the most sensitive barometric pressure sensor we have
  come across. It is capable of detecting the difference in 13cm of air! On top
  of that the MS5637 is low cost and easy to use to boot!

  This library handles the initialization of the MS5637 and is able to
  query the sensor for different readings.

  https://github.com/sparkfun/SparkFun_MS5637_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  MIT License

  Copyright (c) 2016 TE Connectivity

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  SparkFun labored with love to create this code. Feel like supporting open
  source hardware? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14688
*/

#ifndef SPARKFUN_MS5637_ARDUINO_LIBRARY_H
#define SPARKFUN_MS5637_ARDUINO_LIBRARY_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MS5637_COEFFICIENT_COUNT 7

#define MS5637_CONVERSION_TIME_OSR_256 1
#define MS5637_CONVERSION_TIME_OSR_512 2
#define MS5637_CONVERSION_TIME_OSR_1024 3
#define MS5637_CONVERSION_TIME_OSR_2048 5
#define MS5637_CONVERSION_TIME_OSR_4096 9
#define MS5637_CONVERSION_TIME_OSR_8192 17

// Enum
enum ms5637_resolution_osr {
  ms5637_resolution_osr_256 = 0,
  ms5637_resolution_osr_512,
  ms5637_resolution_osr_1024,
  ms5637_resolution_osr_2048,
  ms5637_resolution_osr_4096,
  ms5637_resolution_osr_8192 //5
};

enum ms5637_status {
  ms5637_status_ok,
  ms5637_status_no_i2c_acknowledge,
  ms5637_status_i2c_transfer_error,
  ms5637_status_crc_error
};

enum ms5637_status_code {
  ms5637_STATUS_OK = 0,
  ms5637_STATUS_ERR_OVERFLOW = 1,
  ms5637_STATUS_ERR_TIMEOUT = 4
};

// Functions
class MS5637 {

  public:
    MS5637();

    /**
       \brief Perform initial configuration. Has to be called once.
    */
    boolean begin(TwoWire &wirePort = Wire);

    /**
      \brief Check whether MS5637 device is connected

      \return bool : status of MS5637
            - true : Device is present
            - false : Device is not acknowledging I2C address
    */
    boolean isConnected(void);

    /**
      \brief Reset the MS5637 device

      \return ms5637_status : status of MS5637
            - ms5637_status_ok : I2C transfer completed successfully
            - ms5637_status_i2c_transfer_error : Problem with i2c transfer
            - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum ms5637_status reset(void);

    /**
      \brief Set ADC resolution.

      \param[in] ms5637_resolution_osr : Resolution requested

    */
    void setResolution(enum ms5637_resolution_osr res);

    /**
      \brief Reads the temperature and pressure ADC value and compute the
      compensated values.

      \param[out] float* : Celsius Degree temperature value
      \param[out] float* : mbar pressure value

      \return ms5637_status : status of MS5637
            - ms5637_status_ok : I2C transfer completed successfully
            - ms5637_status_i2c_transfer_error : Problem with i2c transfer
            - ms5637_status_no_i2c_acknowledge : I2C did not acknowledge
            - ms5637_status_crc_error : CRC check error on on the PROM
      coefficients
    */
    enum ms5637_status read_temperature_and_pressure(float *temperature,
        float *pressure);

    float getPressure(); //Returns the latest pressure measurement
    float getTemperature(); //Returns the latest temperature measurement
    double adjustToSeaLevel(double absolutePressure, double actualAltitude);
    double altitudeChange(double currentPressure, double baselinePressure);

  private:
    enum ms5637_status write_command(uint8_t cmd);
    enum ms5637_status read_eeprom_coeff(uint8_t command, uint16_t *coeff);
    boolean crc_check(uint16_t *n_prom, uint8_t crc);
    enum ms5637_status conversion_and_read_adc(uint8_t cmd, uint32_t *adc);
    enum ms5637_status read_eeprom(void);

    uint16_t eeprom_coeff[MS5637_COEFFICIENT_COUNT + 1];
    //bool coeff_read = false;
    enum ms5637_status ms5637_write_command(uint8_t);
    enum ms5637_status ms5637_read_eeprom_coeff(uint8_t, uint16_t *);
    enum ms5637_status ms5637_read_eeprom(void);
    enum ms5637_status ms5637_conversion_and_read_adc(uint8_t, uint32_t *);

    enum ms5637_resolution_osr ms5637_resolution_osr;

    uint8_t conversion_time[6] = {
      MS5637_CONVERSION_TIME_OSR_256,  MS5637_CONVERSION_TIME_OSR_512,
      MS5637_CONVERSION_TIME_OSR_1024, MS5637_CONVERSION_TIME_OSR_2048,
      MS5637_CONVERSION_TIME_OSR_4096, MS5637_CONVERSION_TIME_OSR_8192
    };

    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
    float globalPressure;
    boolean pressureHasBeenRead = true;
    float globalTemperature;
    boolean temperatureHasBeenRead = true;
};
#endif
