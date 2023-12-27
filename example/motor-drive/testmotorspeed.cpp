//can't control motor speed just yet.

#include <Arduino.h>

const int in1Pin = 25;  // H-Bridge input pins
const int in2Pin = 26;
// const int in3Pin = 32;  // H-Bridge pins for second motor
// const int in4Pin = 33;

int higher = 0x1;

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
      digitalWrite(in2Pin,higher);
    }

    else if (ch == 'w') {
      higher += 5;
      Serial.println("increase speed to " + String(higher));
    }

    else if (ch == 's') {
      higher -= 5;
      Serial.println("decrease speed to " + String(higher));
    }

    else if (ch == '-')
    {
      Serial.println("CCW");
      digitalWrite(in1Pin,higher);
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