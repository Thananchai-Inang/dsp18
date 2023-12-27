#include <Arduino.h>

const int in1Pin = 25;  // H-Bridge input pins
const int in2Pin = 26;
// const int in3Pin = 32;  // H-Bridge pins for second motor
// const int in4Pin = 33;

void setup()
{
  Serial.begin(115200);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
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
      Serial.println("CW");
      // first motor
      digitalWrite(in1Pin,LOW);
      digitalWrite(in2Pin,HIGH);
      //second motor
      // digitalWrite(in3Pin,LOW);
      // digitalWrite(in4Pin,HIGH);
    }

    else if (ch == '-')
    {
      Serial.println("CCW");
      digitalWrite(in1Pin,HIGH);
      digitalWrite(in2Pin,LOW);
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