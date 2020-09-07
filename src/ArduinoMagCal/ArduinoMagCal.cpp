////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>

#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "CalLib.h"
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"

// forward definition
void displayMenu();
void doMagMinMaxCal();
void doMagEllipsoidCal();
void processEllipsoid();
void doAccelCal();
bool pollIMU();
void displayMagMinMax();
void displayMagEllipsoid();
void displayAccelMinMax();
char getUserChar();
char get_char();
//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define SERIAL_PORT_SPEED 115200

RTIMUSettings settings; // the settings object

RTIMU *imu; // the IMU object
RTIMUMagCal *magCal;

RTVector3 prev_magMin_ = RTVector3(RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN, RTIMUCALDEFS_DEFAULT_MIN);
RTVector3 prev_magMax_ = RTVector3(RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX, RTIMUCALDEFS_DEFAULT_MAX);

RTIMUAccelCal *accelCal;

RTIMU_DATA imuData;
bool magMinMaxDone;
bool mustExit = false;
bool accelEnables[3];
int accelCurrentAxis;

void setup()
{
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println("ArduinoMagCal starting");

  // Initialize Calibration (Load from EEPROM)
  settings.init();

  imu = RTIMU::createIMU(settings);
  int status_ = 0;
  while ((status_ = imu->IMUInit()) < 0)
  {
    Serial.print("IMU initialization Failed: Error ");
    Serial.println(status_);
    while (true)
    {
      delay(1000);
    }
  }

  //  set up for calibration run
  imu->setCompassCalibrationMode(true);
  imu->setAccelCalibrationMode(true);
  magCal = new RTIMUMagCal(settings);
  magCal->magCalInit();
  magMinMaxDone = false;
  accelCal = new RTIMUAccelCal(settings);
  accelCal->accelCalInit();

  Serial.print("ArduinoIMU calibrating device ");
  Serial.println(imu->IMUName());
}

void loop()
{
  while (!mustExit)
  {
    displayMenu();
    switch (tolower(get_char()))
    {
    case 'x':
      mustExit = true;
      break;

    case 'm':
      doMagMinMaxCal();
      break;

    case 'e':
      doMagEllipsoidCal();
      break;

    case 'a':
      doAccelCal();
      break;
    }
  }
}

void displayMenu()
{
  Serial.printf("\n");
  Serial.printf("Options are: \n\n");
  Serial.printf("  m - calibrate magnetometer with min/max\n");
  Serial.printf("  e - calibrate magnetometer with ellipsoid (do min/max first)\n");
  Serial.printf("  a - calibrate accelerometers\n");
  Serial.printf("  x - exit\n\n");
  Serial.printf("Enter option: ");
}

void doMagMinMaxCal()
{
  unsigned long displayTimer;
  unsigned long now;
  char input;

  magCal->magCalInit();
  magMinMaxDone = false;

  //  now collect data

  Serial.printf("\n\nMagnetometer min/max calibration\n");
  Serial.printf("--------------------------------\n");
  Serial.printf("Waggle the IMU chip around, ensuring that all six axes\n");
  Serial.printf("(+x, -x, +y, -y and +z, -z) go through their extrema.\n");
  Serial.printf("When all extrema have been achieved, enter 's' to save, 'r' to reset\n");
  Serial.printf("or 'x' to abort and discard the data.\n");
  Serial.printf("\nPress any key to start...");
  get_char();

  displayTimer = millis();

  while (1)
  {
    //  poll at the rate recommended by the IMU

    usleep(imu->IMUGetPollInterval() * 1000);

    while (pollIMU())
    {
      magCal->newMinMaxData(imuData.compass);
      now = millis();

      //  display 10 times per second

      if ((now - displayTimer) > 100)
      {
        displayMagMinMax();
        displayTimer = now;
      }
    }

    if ((input = Serial.read()) != 0)
    {
      switch (input)
      {
      case 's':
        Serial.printf("\nSaving min/max data.\n\n");
        magCal->magCalSaveMinMax();
        magMinMaxDone = true;
        return;

      case 'x':
        Serial.printf("\nAborting.\n");
        return;

      case 'r':
        Serial.printf("\nResetting min/max data.\n");
        magCal->magCalReset();
        break;
      }
    }
  }
}

void doMagEllipsoidCal() {}
void processEllipsoid() {}

void doAccelCal()
{
      uint64_t displayTimer;
    uint64_t now;
    char input;

    Serial.printf("\n\nAccelerometer Calibration\n");
    Serial.printf("-------------------------\n");
    Serial.printf("The code normally ignores readings until an axis has been enabled.\n");
    Serial.printf("The idea is to orient the IMU near the current extrema (+x, -x, +y, -y, +z, -z)\n");
    Serial.printf("and then enable the axis, moving the IMU very gently around to find the\n");
    Serial.printf("extreme value. Now disable the axis again so that the IMU can be inverted.\n");
    Serial.printf("When the IMU has been inverted, enable the axis again and find the extreme\n");
    Serial.printf("point. Disable the axis again and press the space bar to move to the next\n");
    Serial.printf("axis and repeat. The software will display the current axis and enable state.\n");
    Serial.printf("Available options are:\n");
    Serial.printf("  e - enable the current axis.\n");
    Serial.printf("  d - disable the current axis.\n");
    Serial.printf("  space bar - move to the next axis (x then y then z then x etc.\n");
    Serial.printf("  r - reset the current axis (if enabled).\n");
    Serial.printf("  s - save the data once all 6 extrema have been collected.\n");
    Serial.printf("  x - abort and discard the data.\n");
    Serial.printf("\nPress any key to start...");
    get_char();

    //  perform all axis reset
    for (int i = 0; i < 3; i++){
        accelCal->accelCalEnable(i, true);
    }
    accelCal->accelCalReset();
    for (int i = 0; i < 3; i++){
        accelCal->accelCalEnable(i, false);
    }
    accelCurrentAxis = 0;

    for (int i = 0; i < 3; i++){
        accelEnables[i] = false;
    }
    displayTimer = millis();

    while (1) {
        //  poll at the rate recommended by the IMU

        usleep(imu->IMUGetPollInterval() * 1000);

        while (pollIMU()) {

            for (int i = 0; i < 3; i++){
                accelCal->accelCalEnable(i, accelEnables[i]);
            }
            accelCal->newAccelCalData(imuData.accel);

            now = millis();

            //  display 10 times per second

            if ((now - displayTimer) > 100) {
                displayAccelMinMax();
                displayTimer = now;
            }
        }

        if ((input = getUserChar()) != 0) {
            switch (input) {
            case 'e' :
                accelEnables[accelCurrentAxis] = true;
                break;

            case 'd' :
                accelEnables[accelCurrentAxis] = false;
                break;

            case 'r' :
                accelCal->accelCalReset();
                break;

            case ' ' :
                if (++accelCurrentAxis == 3){
                    accelCurrentAxis = 0;
                }
                break;

            case 's' :
                if(accelCal->accelCalSave()) {
                  Serial.printf("\nAccelerometer calibration data saved to file.\n");
                }
                else {
                  Serial.printf("\nFAILURE: Accelerometer calibration data NOT saved to EEPROM.\n");
                }
                return;

            case 'x' :
                Serial.printf("\nAborting.\n");
                return;
            }
        }
    }
}
char getUserChar()
{
    char ch = Serial.read();;
    if(ch > 0) {
      return tolower(ch);
    }
    return 0;
}

char get_char()
{
  int ch = 0;
  while (ch <= 0)
  {
    delay(250);
    ch = Serial.read();
  }
  return char(ch);
}


bool pollIMU()
{
  if (imu->IMURead())
  {
    imuData = imu->getIMUData();
    return true;
  }
  else
  {
    return false;
  }
}

void displayMagMinMax()
{
  if (magCal->m_magMin != prev_magMin_ || magCal->m_magMax != prev_magMax_)
  {
    prev_magMin_ = magCal->m_magMin;
    prev_magMax_ = magCal->m_magMax;

    Serial.printf("\n\n");
    Serial.printf("Min x: %6.2f  min y: %6.2f  min z: %6.2f\n", magCal->m_magMin.data(0),
                  magCal->m_magMin.data(1), magCal->m_magMin.data(2));
    Serial.printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\n", magCal->m_magMax.data(0),
                  magCal->m_magMax.data(1), magCal->m_magMax.data(2));
    Serial.flush();
  }
}
void displayMagEllipsoid() {}

void displayAccelMinMax() {
    Serial.printf("\n\n");

    Serial.printf("Current axis: ");
    if (accelCurrentAxis == 0) {
        Serial.printf("x - %s", accelEnables[0] ? "enabled" : "disabled");
        Serial.printf("\nMin x: %6.2f Max x: %6.2f\n", accelCal->m_accelMin.data(0), accelCal->m_accelMax.data(0));
        Serial.printf("AVG x: %6.2f\n", accelCal->m_accel.data(0));

    } else     if (accelCurrentAxis == 1) {
        Serial.printf("y - %s", accelEnables[1] ? "enabled" : "disabled");
        Serial.printf("\nMin y: %6.2f Max y: %6.2f\n", accelCal->m_accelMin.data(1), accelCal->m_accelMax.data(1));
        Serial.printf("AVG y: %6.2f\n", accelCal->m_accel.data(1));

    } else     if (accelCurrentAxis == 2) {
        Serial.printf("z - %s", accelEnables[2] ? "enabled" : "disabled");
        Serial.printf("\nMin z: %6.2f Max z: %6.2f\n", accelCal->m_accelMin.data(2), accelCal->m_accelMax.data(2));
        Serial.printf("AVG z: %6.2f\n", accelCal->m_accel.data(2));
    }


/*
    Serial.printf("\nMin x: %6.2f  min y: %6.2f  min z: %6.2f\n", accelCal->m_accelMin.data(0),
           accelCal->m_accelMin.data(1), accelCal->m_accelMin.data(2));
    Serial.printf("Max x: %6.2f  max y: %6.2f  max z: %6.2f\n", accelCal->m_accelMax.data(0),
           accelCal->m_accelMax.data(1), accelCal->m_accelMax.data(2));
    Serial.flush();
*/
}