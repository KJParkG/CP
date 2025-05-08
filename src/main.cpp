//초음파 센서 모터 제어 코드

#include <Arduino.h>
#include "ESP32Servo.h"

#define SERVO_PIN 17

Servo myservo;

float duration, distance;

#define trigPin 15
#define echoPin 16

int ena = 8;
int in1 = 9;  //IN1
int in2 = 10; //IN2

int in3 = 11; //IN3
int in4 = 12; //IN4
int enb = 13;

float getDistance(){
  /*digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000);
  distance = ((float)(340*duration) / 10000) / 2;
  return distance;*/
  float total = 0;
  int count = 5;
  for (int i = 0; i < count; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    float duration = pulseIn(echoPin, HIGH, 30000);
    total += duration * 0.0343 / 2.0;
    delay(10);  // 너무 빠른 반복 방지
  }
  return total / count;
}
// put function declarations here:
void moveForward(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Go Forward");
}

void moveBackward(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Go Back");
}

void stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void setup() {
  // put your setup code here, to run once:
  myservo.attach(SERVO_PIN);
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  myservo.write(90);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:  
  float frontDistance = getDistance();
  Serial.print("Distance : ");
  Serial.println(frontDistance);
  if (frontDistance > 10){
    moveForward();
  }
  else {
    stop();

    myservo.write(45);
    delay(500);
    float leftDistance = getDistance();
    Serial.println(leftDistance);

    myservo.write(135);
    delay(500);
    float rightDistance = getDistance();
    Serial.println(rightDistance);

    myservo.write(90);

    if(leftDistance > rightDistance && leftDistance > 10){
      //왼쪽으로 방향 전환
      Serial.println("Turn Left");
      delay(1000);
    }
    else if(rightDistance > leftDistance && rightDistance > 10){
      //오른쪽으로 방향 전환
      Serial.println("Turn Right");
      delay(1000);
    }
    else{
      moveBackward();
    }
    
  }

}