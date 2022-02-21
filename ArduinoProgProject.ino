#include "MPU6050.h"
#include"I2Cdev.h"
#include<math.h>
#include <Adafruit_BMP085.h>
#define kaki 3.28084
Adafruit_BMP085 bmp;
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float pitch,roll,yaw;
float Totalres;
double Temp,Press,Alt,Altf,p;
void setup() {
  Wire.begin();
  Serial.begin(9600);
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  Temp=bmp.readTemperature();
  Press=bmp.readPressure();
  p=(Press/1000);//KPa
  Alt=bmp.readAltitude(); //meter
  Altf=Alt*kaki; //feet
  accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  Totalres=sqrt(pow(ax,2)+pow(ay,2)+pow(az,2));
  roll=asin((float)ay/Totalres)*-57.296;
  pitch=asin((float)ax/Totalres)*-57.296;
  yaw=asin((float)az/Totalres)*-57.296;
  Serial.print((int)roll);//pitch
  Serial.print(",");
  Serial.print((int)pitch);//roll
  Serial.print(",");
  Serial.print((int)yaw);//yaw
  Serial.print(",");
  Serial.print(ax);//acc x data 2
  Serial.print(",");
  Serial.print(ay);//acc Y data 3
  Serial.print(",");
  Serial.print(az);//acc Z data 4
  Serial.print(",");
  Serial.print(gx);//gyro X data 5
  Serial.print(",");
  Serial.print(gy);//gyro Y 
  Serial.print(",");
  Serial.print(gz);//gyro Z 
  Serial.print(",");
  Serial.print(Temp);
  Serial.print(",");
  Serial.print(p);
  Serial.print(",");
  Serial.println(Altf);
  delay(500);
}
