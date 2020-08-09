/*
  TinyCircuits LSM9DS1 9 Axis TinyShield Example Sketch
  
  This demo is intended for the ASD2511 Sensor Board TinyShield with a LSM9DS1
  9 axis sensor populated. It shows basic use of a modified RTIMULib with the
  sensor.
  
  This program now includes an EEPROM compatibility file for TinyScreen+.
  Using it will lock the last 16KB of flash for EEPROM emulation and prevent
  the bootloader from erasing or writing that section. This should not affect
  other programs unless they are trying to use the last 16KB of flash.
  Written 11 July 2016
  By Ben Rose
  Modified 07 January 2019
  By Hunter Hykes
  https://TinyCircuits.com
*/

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
////////////////////////////////////////////////////////////////////////////


#include <Wire.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#ifndef ARDUINO_ARCH_SAMD
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <Time.h>
#include <TimeLib.h>
#endif

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  1                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

#if defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitor SerialUSB
#else
  #define SerialMonitor Serial
#endif

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

const int chipSelect = 10;
void setup()
{
    int errcode;
  
    SerialMonitor.begin(SERIAL_PORT_SPEED);
    while(!SerialMonitor);
    
    Wire.begin();
    imu = RTIMU::createIMU(&settings);                        // create the imu object
  
    SerialMonitor.print("ArduinoIMU starting using device "); SerialMonitor.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        SerialMonitor.print("Failed to init IMU: "); SerialMonitor.println(errcode);
    }

    if (imu->getCalibrationValid())
        SerialMonitor.println("Using compass calibration");
    else
        SerialMonitor.println("No valid compass calibration data");
    
    lastDisplay = lastRate = millis();
    sampleCount = 0;

    
    
    pinMode(chipSelect, OUTPUT);
    if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
        // don't do anything more:
        while (1);
    }
    //File IMU_LOG = SD.open("IMU_Log.txt", FILE_WRITE);
    //if (IMU_LOG){
    //  Serial.println("New Session...");
    //  IMU_LOG.print("New Session...");
    //  IMU_LOG.close();
    //}
    
    //File IMU_LOG = SD.open("IMU_Log.txt", FILE_WRITE);
    //IMU_LOG.print("Accel X [mm]");IMU_LOG.print(",");IMU_LOG.print("Accel Y [mm]");IMU_LOG.print(",");IMU_LOG.print("Accel Z [mm]");IMU_LOG.print(",");
    //IMU_LOG.print("Gyro X [Deg]");IMU_LOG.print(",");IMU_LOG.print("Gyro Y [Deg]");IMU_LOG.print(",");IMU_LOG.print("Gyro Z [Deg]");IMU_LOG.print(",");
    //IMU_LOG.print("Mag X");IMU_LOG.print(",");IMU_LOG.print("Mag Y");IMU_LOG.print(",");IMU_LOG.print("Mag Z");IMU_LOG.print(",");
    //IMU_LOG.print("FUSEPos X [Deg]");IMU_LOG.print(",");IMU_LOG.print("FUSEPos Y [Deg]");IMU_LOG.print(",");IMU_LOG.print("FUSEPos Z [Deg]");IMU_LOG.print("\n");
    //IMU_LOG.close();
   
    
    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.5);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
}

void loop()
{  
  unsigned long now = millis();
  unsigned long delta;
  
  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;   
    
    if ((delta = now - lastRate) >= 1000) {
      SerialMonitor.print("Sample rate: "); SerialMonitor.print(sampleCount);
      if (imu->IMUGyroBiasValid())
        SerialMonitor.println(", gyro bias valid");
      else
        SerialMonitor.println(", calculating gyro bias - don't move IMU!!");
        
      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      RTVector3 accelData=imu->getAccel();
      RTVector3 gyroData=imu->getGyro();
      RTVector3 compassData=imu->getCompass();
      RTVector3 fusionData=fusion.getFusionPose();
      write2card(accelData.x(),accelData.y(),accelData.z(),gyroData.x(),gyroData.y(), gyroData.z(),compassData.x(),compassData.y(),compassData.z(),fusionData.x(),fusionData.y(),fusionData.z());
      //displayAxis("Accel:", accelData.x(), accelData.y(), accelData.z());        // accel data
      //displayAxis("Gyro:", gyroData.x(), gyroData.y(), gyroData.z());            // gyro data
      //displayAxis("Mag:", compassData.x(), compassData.y(), compassData.z());    // compass data
      //displayDegrees("Pose:", fusionData.x(), fusionData.y(), fusionData.z());   // fused output
      //SerialMonitor.println();
    }
  }
}

void displayAxis(const char *label, float x, float y, float z)
{
  SerialMonitor.print(label);
  SerialMonitor.print(" x:"); SerialMonitor.print(x);
  SerialMonitor.print(" y:"); SerialMonitor.print(y);
  SerialMonitor.print(" z:"); SerialMonitor.print(z);
}

void write2card(float Ax, float Ay, float Az,float Gx, float Gy, float Gz,float Mx, float My, float Mz, float Px, float Py, float Pz)
{
  File IMU_LOG = SD.open("IMU_Log.txt", FILE_WRITE);
    
  if (IMU_LOG){
  
  SerialMonitor.print("Writing to Card \n");  
  //SerialMonitor.print(Ax);SerialMonitor.print(",");SerialMonitor.print(Ay);SerialMonitor.print(",");SerialMonitor.print(Az);SerialMonitor.print(",");
  //SerialMonitor.print(Gx);SerialMonitor.print(",");SerialMonitor.print(Gy);SerialMonitor.print(",");SerialMonitor.print(Gz);SerialMonitor.print(",");
  //SerialMonitor.print(Mx);SerialMonitor.print(",");SerialMonitor.print(My);SerialMonitor.print(",");SerialMonitor.print(Mz);SerialMonitor.print(",");
  //SerialMonitor.print(Px);SerialMonitor.print(",");SerialMonitor.print(Py);SerialMonitor.print(",");SerialMonitor.print(Pz);
  //SerialMonitor.print("\n");

  //SerialMonitor.print(millis()); SerialMonitor.print("ms \n");


  IMU_LOG.print(millis());IMU_LOG.print(",");
  IMU_LOG.print(Ax);IMU_LOG.print(",");IMU_LOG.print(Ay);IMU_LOG.print(",");IMU_LOG.print(Az);IMU_LOG.print(",");
  IMU_LOG.print(Gx * RTMATH_RAD_TO_DEGREE);IMU_LOG.print(",");IMU_LOG.print(Gy * RTMATH_RAD_TO_DEGREE);IMU_LOG.print(",");IMU_LOG.print(Gz * RTMATH_RAD_TO_DEGREE);IMU_LOG.print(",");
  IMU_LOG.print(Mx);IMU_LOG.print(",");IMU_LOG.print(My);IMU_LOG.print(",");IMU_LOG.print(Mz);IMU_LOG.print(",");
  IMU_LOG.print(Px * RTMATH_RAD_TO_DEGREE );IMU_LOG.print(",");IMU_LOG.print(Py * RTMATH_RAD_TO_DEGREE);IMU_LOG.print(",");IMU_LOG.print(Pz * RTMATH_RAD_TO_DEGREE);IMU_LOG.print("\n");
  IMU_LOG.close();
  
  }
  else{
  Serial.println("error opening test.txt");
  //SerialMonitor.print(Ax);SerialMonitor.print(",");SerialMonitor.print(Ay);SerialMonitor.print(",");SerialMonitor.print(Az);SerialMonitor.print(",");
  //SerialMonitor.print(Gx);SerialMonitor.print(",");SerialMonitor.print(Gy);SerialMonitor.print(",");SerialMonitor.print(Gz);SerialMonitor.print(",");
  //SerialMonitor.print(Mx);SerialMonitor.print(",");SerialMonitor.print(My);SerialMonitor.print(",");SerialMonitor.print(Mz);SerialMonitor.print(",");
  //SerialMonitor.print(Px);SerialMonitor.print(",");SerialMonitor.print(Py);SerialMonitor.print(",");SerialMonitor.print(Pz);
  //SerialMonitor.print("\n");
    IMU_LOG.close();
  }
}

void displayDegrees(const char *label, float x, float y, float z)
{
  SerialMonitor.print(label);
  SerialMonitor.print(" x:"); SerialMonitor.print(x * RTMATH_RAD_TO_DEGREE);
  SerialMonitor.print(" y:"); SerialMonitor.print(y * RTMATH_RAD_TO_DEGREE);
  SerialMonitor.print(" z:"); SerialMonitor.print(z * RTMATH_RAD_TO_DEGREE);
}
