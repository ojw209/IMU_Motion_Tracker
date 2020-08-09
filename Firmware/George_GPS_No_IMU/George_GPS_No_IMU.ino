/*
   This Arduino sketch will log GPS NMEA data to a SD card every second
*/

#include <SoftwareSerial.h>
#include <SD.h>
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include <Time.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
// #include <DateTime.h>

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;  

tmElements_t tm;

int ts=0;
int roll=0;
int pitch=0;
int yaw=0;
int ror=0;
int rop=0;
int roy=0;
int ax=0;
int ay=0;
int az=0;
int cx=0;
int cy=0;
int cz=0;
int Sec1=00;
int Sec2=20;
int Sec3=30;
int Hour=15;
int Min=36;
int Sec=00;
int timecount=0;
int timeon=60000;

unsigned long lastDisplay; 


int inByte = 0;         // incoming serial byte
byte pbyGpsBuffer[100];
int byBufferIndex = 0;

String readString;
String gpsstring;
int a = 0;
String str;
String content= "";
char c;
// The GPS connection is attached with a software serial port
SoftwareSerial Gps_serial(A1, A0);

static const unsigned long REFRESH_INTERVAL = 25000; // ms
static const unsigned long REFRESH_INTERVAL2 = 30000; // ms
static unsigned long lastRefreshTime = 0;


void setup()
{   

  // Init the GPS Module to wake mode
  pinMode(A2, INPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW );   
  delay(5); 
  if( digitalRead(A2) == LOW )
  {
    // Need to wake the module
    digitalWrite(A3, HIGH ); 
    delay(5); 
    digitalWrite(A3, LOW );      
  } 
        
  
  Serial.begin(9600);
  Gps_serial.begin(9600);  
  
  Wire.begin();
  imu = RTIMU::createIMU(&settings);
  imu->IMUInit();
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
  fusion.setSlerpPower(0.5);
   
  // Serial.print("Initializing SD card...");
  
  if (SD.begin()) {
    // Serial.println("SD card is ready to use.");
  } 
  else {
    // Serial.println("SD card initialization failed");
    return;
  }
  
}




void loop()
{
 
  imudata();
  
  while (millis() - lastRefreshTime >= REFRESH_INTERVAL)
  {
    gpsdata();
    if (millis() - lastRefreshTime >= REFRESH_INTERVAL2)
    {
      lastRefreshTime += REFRESH_INTERVAL2;
    }
  }


 
}



void gpsdata()
{
  File gpsfile = SD.open("gps.csv", FILE_WRITE);
  while (Gps_serial.available())
    {
      
      c = Gps_serial.read();
      // Serial.write(c);
      // if (c=='\n') 
      // {
      //   Serial.print(millis()); Serial.print(","); // After end-of-line print millis()
      // }

      if (gpsfile) 
      {
        if (c=='\n') 
        {
          gpsfile.print(millis()); gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.print(",");gpsfile.println(",");

          // gpsimu.print(millis()); gpsimu.print(","); // After end-of-line print millis()
        }
          
        gpsfile.write(c); 
        
      } 
        
    }  
  
    gpsfile.close();   

}

void imudata()
{
  File imufile = SD.open("imu.csv", FILE_WRITE);
  
  

  if (imu->IMURead()) 
  {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    
    ts = imu->getTimestamp();
    
    roll=fusion.getFusionPose().x(); // left roll is negative
    pitch=fusion.getFusionPose().y(); // nose down is negative
    yaw=fusion.getFusionPose().z(); // 0 Yaw = 270 magnetic, this gives left or right up to 180 degrees
    // float hdm = yaw-90;        // converts yaw to heading magnetic  
    // if (yaw < 90 && yaw >= -179.99) {  
    // hdm = yaw+270;  
    // } 
    ror=imu->getGyro().x(); // rate of roll
    rop=imu->getGyro().y(); // rate of pitch
    roy=imu->getGyro().z(); // rate of yaw
    
    ax=imu->getAccel().x(); // acceleration on the x-axis 
    ay=imu->getAccel().y(); // acceleration on the y-axis 
    az=imu->getAccel().z(); // acceleration on the z-axis
    
    cx=imu->getCompass().x();
    cy=imu->getCompass().y();
    cz=imu->getCompass().z();
    
  } 
    
    Serial.print(millis()); Serial.print(",");
    // Serial.print(ts); Serial.print(",");
    Serial.print(roll); Serial.print(",");
    Serial.print(pitch); Serial.print(",");
    Serial.print(yaw); Serial.print(",");
    Serial.print(ror); Serial.print(",");
    Serial.print(rop); Serial.print(",");
    Serial.print(roy); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(cx); Serial.print(",");
    Serial.print(cy); Serial.print(",");
    Serial.print(cz); Serial.println(",");

  if (imufile) 
  {
    imufile.print(millis()); imufile.print(",");
    // gpsimu.print(ts); gpsimu.print(",");
    imufile.print(roll); imufile.print(",");
    imufile.print(pitch); imufile.print(",");
    imufile.print(yaw); imufile.print(",");
    imufile.print(ror); imufile.print(",");
    imufile.print(rop); imufile.print(",");
    imufile.print(roy); imufile.print(",");
    imufile.print(ax); imufile.print(",");
    imufile.print(ay); imufile.print(",");
    imufile.print(az); imufile.print(",");
    imufile.print(cx); imufile.print(",");
    imufile.print(cy); imufile.print(",");
    imufile.print(cz); imufile.println(",");
  }
  imufile.close();
  
  
  
}
