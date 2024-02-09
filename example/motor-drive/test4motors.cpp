#include <Arduino.h>

const int pump1Pin1 = 25;  // IN1 of motor drive L
const int pump1Pin2 = 26;  // IN2 of motor drive L
const int pump1PinPWM = 27;  
const int pwmChannelPump1 = 1; //define pump 1 as channel 1

//-----------------------------pump 2 pins------------------------------------
const int pump2Pin1 = 4;  // IN1 of motor drive R
const int pump2Pin2 = 2;  // IN2 of motor drive R
const int pump2PinPWM = 15;  
const int pwmChannelPump2 = 2; //define pump 2 as channel 2

//-----------------------------pump 3 pins------------------------------------
const int pump3Pin1 = 13;  // IN3 of motor drive L
const int pump3Pin2 = 12;  // IN4 of motor drive L
const int pump3PinPWM = 14;  
const int pwmChannelPump3 = 3; //define pump 3 as channel 3

//-----------------------------pump 4 pins------------------------------------
const int pump4Pin1 = 16;  // IN3 of motor drive R
const int pump4Pin2 = 33;  // IN4 of motor drive R
const int pump4PinPWM = 32;  
const int pwmChannelPump4 = 4; //define pump 4 as channel 4

//other PWM param
const int freq = 30000;
const int resolutionPump = 8;
const float PWM1 = 200;
const float PWM2 = 200;


void setup()
{
  Serial.begin(115200);
  //-----------------------------pump 1 setup------------------------------------
    pinMode(pump1Pin1, OUTPUT); //motor 1
    pinMode(pump1Pin2, OUTPUT);
    pinMode(pump1PinPWM, OUTPUT);
    ledcSetup(pwmChannelPump1, freq, resolutionPump);
    ledcAttachPin(pump1PinPWM, pwmChannelPump1); //map pwm channel 1 to motor1PinPWM

    //-----------------------------pump 2 setup------------------------------------
    pinMode(pump2Pin1, OUTPUT); //motor 2
    pinMode(pump2Pin2, OUTPUT);
    pinMode(pump2PinPWM, OUTPUT);
    ledcSetup(pwmChannelPump2, freq, resolutionPump);
    ledcAttachPin(pump2PinPWM, pwmChannelPump2); //map pwm channel 2 to motor2PinPWM
    //-----------------------------pump 3 setup------------------------------------
    pinMode(pump3Pin1, OUTPUT); //motor 3
    pinMode(pump3Pin2, OUTPUT);
    pinMode(pump3PinPWM, OUTPUT);
    ledcSetup(pwmChannelPump3, freq, resolutionPump);
    ledcAttachPin(pump3PinPWM, pwmChannelPump3); //map pwm channel 3 to motor3PinPWM

    //-----------------------------pump 4 setup------------------------------------
    pinMode(pump4Pin1, OUTPUT); //motor 4
    pinMode(pump4Pin2, OUTPUT);
    pinMode(pump4PinPWM, OUTPUT);
    ledcSetup(pwmChannelPump4, freq, resolutionPump);
    ledcAttachPin(pump4PinPWM, pwmChannelPump4); //map pwm channel 4 to motor4PinPWM

  Serial.println("+ - sets direction of motors, any other key stops motors");
}

void loop()
{
  if ( Serial.available()) {
    char ch = Serial.read();
    if (ch == '1')
    {
      Serial.println("pump1out"); //pumpout
      digitalWrite(pump1Pin1,LOW);
      digitalWrite(pump1Pin2,HIGH);
      ledcWrite(pwmChannelPump1, PWM1);
    }

    else if (ch == 'q')
    {
      Serial.println("pump1in"); //pumpin
      digitalWrite(pump1Pin1,HIGH);
      digitalWrite(pump1Pin2,LOW);
      ledcWrite(pwmChannelPump1, PWM2);
    }

    else if (ch == '2')
    {
      Serial.println("pump2out"); //pumpout
      digitalWrite(pump2Pin1,LOW);
      digitalWrite(pump2Pin2,HIGH);
      ledcWrite(pwmChannelPump2, PWM1);
    }

    else if (ch == 'w')
    {
      Serial.println("pump2in"); //pumpin
      digitalWrite(pump2Pin1,HIGH);
      digitalWrite(pump2Pin2,LOW);
      ledcWrite(pwmChannelPump2, PWM2);
    }

    else if (ch == '3')
    {
      Serial.println("pump3out"); //pumpout
      digitalWrite(pump3Pin1,LOW);
      digitalWrite(pump3Pin2,HIGH);
      ledcWrite(pwmChannelPump3, PWM1);
    }

    else if (ch == 'e')
    {
      Serial.println("pump3in"); //pumpin
      digitalWrite(pump3Pin1,HIGH);
      digitalWrite(pump3Pin2,LOW);
      ledcWrite(pwmChannelPump3, PWM2);
    }

    else if (ch == '4')
    {
      Serial.println("pump4out"); //pumpout
      digitalWrite(pump4Pin1,LOW);
      digitalWrite(pump4Pin2,HIGH);
      ledcWrite(pwmChannelPump4, PWM1);
    }

    else if (ch == 'r')
    {
      Serial.println("pump4in"); //pumpin
      digitalWrite(pump4Pin1,HIGH);
      digitalWrite(pump4Pin2,LOW);
      ledcWrite(pwmChannelPump4, PWM2);
    }

    else
    {
      Serial.println("Stop motors");
      digitalWrite(pump1Pin1,LOW);
      digitalWrite(pump1Pin2,LOW);
      digitalWrite(pump2Pin1,LOW);
      digitalWrite(pump2Pin2,LOW);
      digitalWrite(pump3Pin1,LOW);
      digitalWrite(pump3Pin2,LOW);
      digitalWrite(pump4Pin1,LOW);
      digitalWrite(pump4Pin2,LOW);
    }
  }
  delay(100);
}