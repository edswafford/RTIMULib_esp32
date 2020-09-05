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

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define SERIAL_PORT_SPEED 115200

RTIMUSettings settings; // the settings object

RTIMU *imu; // the IMU object
RTIMUMagCal *magCal;
RTIMUAccelCal *accelCal;

RTIMU_DATA imuData;
bool magMinMaxDone;
bool mustExit = false;
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
  while (true)
  {
    delay(1000);
  }

  /*  boolean changed;
  RTVector3 mag;
  
  if (imu->IMURead()) {                                 // get the latest data
    changed = false;
    mag = imu->getCompass();
    for (int i = 0; i < 3; i++) {
      if (mag.data(i) < calData.magMin[i]) {
        calData.magMin[i] = mag.data(i);
        changed = true;
      }
      if (mag.data(i) > calData.magMax[i]) {
        calData.magMax[i] = mag.data(i);
        changed = true;
      }
    }
 
    if (changed) {
      Serial.println("-------");
      Serial.print("minX: "); Serial.print(calData.magMin[0]);
      Serial.print(" maxX: "); Serial.print(calData.magMax[0]); Serial.println();
      Serial.print("minY: "); Serial.print(calData.magMin[1]);
      Serial.print(" maxY: "); Serial.print(calData.magMax[1]); Serial.println();
      Serial.print("minZ: "); Serial.print(calData.magMin[2]);
      Serial.print(" maxZ: "); Serial.print(calData.magMax[2]); Serial.println();
    }
  }
  
  if (Serial.available()) {
    if (Serial.read() == 's') {                  // save the data
      calData.magValid = true;
      calLibWrite(0, &calData);
      Serial.print("Mag cal data saved for device "); Serial.println(imu->IMUName());
    }
  }

  */
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

      if ((now - displayTimer) > 100000)
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
        printf("\nSaving min/max data.\n\n");
        magCal->magCalSaveMinMax();
        magMinMaxDone = true;
        return;

      case 'x':
        printf("\nAborting.\n");
        return;

      case 'r':
        printf("\nResetting min/max data.\n");
        magCal->magCalReset();
        break;
      }
    }
  }
}

void doMagEllipsoidCal() {}
void processEllipsoid() {}
void doAccelCal() {}

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

void displayMagMinMax() {}
void displayMagEllipsoid() {}
void displayAccelMinMax() {}