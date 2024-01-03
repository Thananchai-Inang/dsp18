#include <Arduino.h>
#include <SPI.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

//----------------------------Gyroscope parameters----------------------------
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int valx , valy , valz;
char rd;
int prevVal;
int pin11 = 5 , pin10 = 4 ;
int val1 , val2 ;
int valgy1 = 0 , valgy2 = 0;

//----------------------------Depth Sensor parameters----------------------------
int pwmChannel = 0;
int frequence = 32768;
int resolution = 8;
int pwmPin = 5;
int pH2O = 1;
int pHG = 13.56;
bool init_PAir = false;
float PAir = 0.0;
//----------------------------Depth sensor function----------------------------
void resetsensor() {
  SPI.setDataMode(SPI_MODE0); 
  SPI.transfer(0x15);
  SPI.transfer(0x55);
  SPI.transfer(0x40);
}



void setup() {
  Serial.begin(115200);

  //----------------------------Depth Sensor----------------------------
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  ledcSetup(pwmChannel, frequence, resolution);
  ledcAttachPin(pwmPin, pwmChannel);

  //----------------------------Gyroscope----------------------------
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Initialize MPU");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
  }



void loop() {
  //----------------------------main Depth Sensor----------------------------
  ledcWrite(pwmChannel, 127);
  resetsensor();
  unsigned int result1 = 0;
  unsigned int inbyte1 = 0;
  SPI.transfer(0x1D); 
  SPI.transfer(0x50);
  SPI.setDataMode(SPI_MODE1);
  result1 = SPI.transfer(0x00);
  result1 = result1 << 8;
  inbyte1 = SPI.transfer(0x00);
  result1 = result1 | inbyte1;
  resetsensor(); 
  unsigned int result2 = 0;
  byte inbyte2 = 0; 
  SPI.transfer(0x1D);
  SPI.transfer(0x60);
  SPI.setDataMode(SPI_MODE1); 
  result2 = SPI.transfer(0x00);
  result2 = result2 <<8;
  inbyte2 = SPI.transfer(0x00);
  result2 = result2 | inbyte2;
  resetsensor();
  unsigned int result3 = 0;
  byte inbyte3 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x90); 
  SPI.setDataMode(SPI_MODE1); 
  result3 = SPI.transfer(0x00);
  result3 = result3 <<8;
  inbyte3 = SPI.transfer(0x00);
  result3 = result3 | inbyte3;
  resetsensor();
  unsigned int result4 = 0;
  byte inbyte4 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0xA0);
  SPI.setDataMode(SPI_MODE1); 
  result4 = SPI.transfer(0x00);
  result4 = result4 <<8;
  inbyte4 = SPI.transfer(0x00);
  result4 = result4 | inbyte4;
  long c1 = (result1 >> 1) & 0x7FFF;
  long c2 = ((result3 & 0x003F) << 6) | (result4 & 0x003F);
  long c3 = (result4 >> 6) & 0x03FF;
  long c4 = (result3 >> 6) & 0x03FF;
  long c5 = ((result1 & 0x0001) << 10) | ((result2 >> 6) & 0x03FF);
  long c6 = result2 & 0x003F;
  resetsensor();

  //----------------------------main Pressure----------------------------
  unsigned int presMSB = 0; 
  unsigned int presLSB = 0; 
  unsigned int D1 = 0;
  SPI.transfer(0x0F);
  SPI.transfer(0x40);
  delay(35);
  SPI.setDataMode(SPI_MODE1);
  presMSB = SPI.transfer(0x00);
  presMSB = presMSB << 8;
  presLSB = SPI.transfer(0x00);
  D1 = presMSB | presLSB;
  resetsensor();

  //----------------------------main Temperature----------------------------
  unsigned int tempMSB = 0;
  unsigned int tempLSB = 0;
  unsigned int D2 = 0;
  SPI.transfer(0x0F);
  SPI.transfer(0x20);
  delay(35);
  SPI.setDataMode(SPI_MODE1);
  tempMSB = SPI.transfer(0x00);
  tempMSB = tempMSB << 8;
  tempLSB = SPI.transfer(0x00);
  D2 = tempMSB | tempLSB;
  const long UT1 = (c5 << 3) + 20224;
  const long dT = D2 - UT1;
  const long TEMP = 200 + ((dT * (c6 + 50)) >> 10);
  const long OFF  = (c2 * 4) + (((c4 - 512) * dT) >> 12);
  const long SENS = c1 + ((c3 * dT) >> 10) + 24576;
  const long X = (SENS * (D1 - 7168) >> 14) - OFF;
  long PCOMP = ((X * 10) >> 5) + 2500;
  float TEMPREAL = TEMP/10;
  //----------------------------Initialize the pressure sensor----------------------------
  if (!init_PAir) {
    PAir = PCOMP * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06
    init_PAir = true;
    Serial.println("Initialize PAir variable");
  }
  //----------------------------Calculate Depth----------------------------
  float PCOMPHG = PCOMP * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06  
  float Depth = (PCOMPHG-PAir)*(pHG/pH2O)/10;
    
  //----------------------------print Depth----------------------------
  Serial.print("Compensated pressure in mmHg = ");
  Serial.println(PCOMPHG);
  Serial.print("Depth in cm = ");
  Serial.println(Depth);

  //----------------------------print Gyroscope Value----------------------------
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

  delay(200);
}

