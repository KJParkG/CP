#include <Arduino.h>
#include "ESP32Servo.h"

#define SERVO_PIN 17

Servo myservo;

#define trigPin 15
#define echoPin 16
int angle = 0;
// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  myservo.attach(SERVO_PIN);
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  float duration, distance;

  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = ((float)(340*duration) / 10000) / 2;

  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println("cm");

  if (distance < 5) {
      angle = 0;
      Serial.println("5 이하");
      delay(500);
  }
  else{
    angle = 180;
    delay(500);
  }
  myservo.write(angle);
}