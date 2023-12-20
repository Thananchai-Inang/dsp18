#include <SPI.h>

//Pump
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14; 
//PWM
const int freq = 30000;
const int pwmChannelPump = 0;
const int resolutionPump = 8;
int dutyCycle = 120;
float controlP; 

//Solenoid
int valvepin = 17;

//Depth Sensor
int pwmChannel = 0;
int frequence = 32768;
int resolution = 8;
int pwmPin = 21;
int pH2O = 1;
int pHG = 13.56;
int PAir = 752.31; //แก้ค่าก่อนเริ่ม

//Control
float kp = 6;
float kd = 1;
float err;
float Lasterr = 0;
float der;
float Depth;

float DepthTarget = 20;////////target///////////

float LastDepth = 0;
unsigned long previousTime = 0;

//plotter
int low = 0;
int high = 40;

void resetsensor() {
  SPI.setDataMode(SPI_MODE0); 
  SPI.transfer(0x15);
  SPI.transfer(0x55);
  SPI.transfer(0x40);
}

void setup() {
  Serial.begin(115200);
//Pump
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  ledcSetup(pwmChannelPump, freq, resolutionPump);
  ledcAttachPin(enable1Pin, pwmChannelPump);
  
//Solenoid
  pinMode(valvepin, OUTPUT);

//Depth Sensor
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
    ledcSetup(pwmChannel, frequence, resolution);
    ledcAttachPin(pwmPin, pwmChannel);
}

float readDepth() {
  //Depth Sensor
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
  //Pressure:
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
  //Temperature:
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
  float PCOMPHG = PCOMP * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06
  float Depth = (PCOMPHG-PAir)*(pHG/pH2O)/10;

  return Depth;
}

void loop() {
unsigned long currentTime = millis();
unsigned long timeDifference = currentTime - previousTime;   
float Depth=readDepth();

//Control
  err = DepthTarget - Depth;
  
  // derivative = (err - lasterr)/(time-lasttime);
  der = (err - Lasterr)/(timeDifference);

  previousTime = currentTime;
  Lasterr = err;

  controlP = float((kp * err)+(kd * der));

  int PWM = map(abs(controlP),0,190,140,255);

  if (controlP >= 0) {
    digitalWrite(valvepin, LOW);
    digitalWrite(motor1Pin1, LOW); //สูบน้ำเข้า
    digitalWrite(motor1Pin2, HIGH);
    ledcWrite(pwmChannelPump, PWM);
    Serial.print("Pump_IN with PWM: ");
    Serial.println(PWM);
  } 
  else {
    digitalWrite(valvepin, LOW);
    digitalWrite(motor1Pin1, HIGH); //สูบน้ำออก
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannelPump, PWM);
    Serial.print("Pump_OUT with PWM: ");
    Serial.println(PWM);
  }
  Serial.print("low:");
  Serial.print(low); // To freeze the lower limit
  Serial.print(",");
  Serial.print("high:");
  Serial.print(high); // To freeze the upper limit
  Serial.print(",");
  Serial.print("err:");
  Serial.print(err);
  Serial.print(",");
  Serial.print("Depth:");
  Serial.print(Depth);
  Serial.print(",");
  Serial.print("Depth target:");
  Serial.println(DepthTarget);
  // Serial.print(",");
  // Serial.print("Der:");
  // Serial.print(der);
  // Serial.print(",");
  // Serial.print("controlP:");
  // Serial.println(controlP);
}