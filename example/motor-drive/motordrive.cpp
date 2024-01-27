#include <Arduino.h>

const int in1Pin = 25;  // H-Bridge input pins
const int in2Pin = 26;
const int pump1PinPWM = 27;
const int pwmChannelPump1 = 1;
//other PWM param
const int freq = 30000;
const int resolutionPump = 8;
const float PWM1 = 150;
const float PWM2 = 200;


void setup()
{
  Serial.begin(115200);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(pump1PinPWM, OUTPUT);
  ledcSetup(pwmChannelPump1, freq, resolutionPump);
  ledcAttachPin(pump1PinPWM, pwmChannelPump1); //map pwm channel 1 to motor1PinPWM
  // pinMode(in3Pin, OUTPUT);
  // pinMode(in4Pin, OUTPUT);
  Serial.println("+ - sets direction of motors, any other key stops motors");
}

void loop()
{
  if ( Serial.available()) {
    char ch = Serial.read();
    if (ch == '+')
    {
      Serial.println("CW"); //pumpout
      // first motor
      digitalWrite(in1Pin,LOW);
      digitalWrite(in2Pin,HIGH);
      ledcWrite(pwmChannelPump1, PWM1);
      //second motor
      // digitalWrite(in3Pin,LOW);
      // digitalWrite(in4Pin,HIGH);
    }

    else if (ch == '-')
    {
      Serial.println("CCW"); //pumpin
      digitalWrite(in1Pin,HIGH);
      digitalWrite(in2Pin,LOW);
      ledcWrite(pwmChannelPump1, PWM2);
      // digitalWrite(in3Pin,HIGH);
      // digitalWrite(in4Pin,LOW);
    }

    else
    {
      Serial.println("Stop motors");
      digitalWrite(in1Pin,LOW);
      digitalWrite(in2Pin,LOW);
      // digitalWrite(in3Pin,LOW);
      // digitalWrite(in4Pin,LOW);
    }
  }
  delay(100);
}