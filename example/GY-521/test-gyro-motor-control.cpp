#include <Arduino.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int valx , valy , valz;
char rd;
int prevVal;
int ledR = 16 ;
int ledG = 14 ;
int ledY = 12 ;
int pin11 = 5 , pin10 = 4 ;
int val1 , val2 ;
int valgy1 = 0 , valgy2 = 0;

void setup() {
  pinMode(ledR, OUTPUT) ;
  pinMode(ledG, OUTPUT) ;
  pinMode(ledY, OUTPUT) ;
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  valx = map(ax, -17000, 17000, 0, 179);
  valy = map(ay, -17000, 17000, 0, 179);
  valz = map(az, -17000, 17000, 0, 179);
  Serial.print("axis x = ") ;
  Serial.print(valx) ;
  Serial.print(" axis y = ") ;
  Serial.print(valy) ;
  Serial.print(" axis z = ") ;
  Serial.println(valz) ;

    
  delay(100);
}