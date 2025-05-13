//초음파 센서 모터 제어 코드

#include <Arduino.h>
#include "ESP32Servo.h"

#define SERVO_PIN 17

Servo myservo;

float duration, distance;
float backdistance; // 후방 거리

#define trigPin 15
#define echoPin 16
#define backTrigPin 6
#define backEchoPin 7

#define TURN_TIME 1000 // 실제 회전시간에 맞게 조절

int ena = 3;
int in1 = 4;  //IN1
int in2 = 5; //IN2

int in3 = 12; //IN3
int in4 = 13; //IN4
int enb = 18;

float measureDistance(int trig, int echo){
  //초음파센서 거리 측정 함수
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH, 30000);
  distance = ((float)(340*duration) / 10000) / 2; // 거리 = (음속 340 m/s * 시간(us)) / 2 / 10000 → cm로 변환
  delay(10);
  return distance;

  //5번 실행하여 평균 구하는 함수
  /*float total = 0;
  int count = 5;
  for (int i = 0; i < count; i++) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    float duration = pulseIn(echo, HIGH, 30000);
    total += duration * 0.0343 / 2.0;
    delay(10);  // 너무 빠른 반복 방지
  }
  return total / count;*/
}
float getDistance(){
  return measureDistance(trigPin, echoPin);
}

float getBackDistance(){
  return measureDistance(backTrigPin, backEchoPin);
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
  Serial.println("Stop");
}

void turnLeft(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Turn Left");
  //좌회전
}

void turnRight(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Turn Right");
  //우회전
}

void setup() {
  // put your setup code here, to run once:
  myservo.attach(SERVO_PIN);
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(backTrigPin, OUTPUT);
  pinMode(backEchoPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  myservo.write(90);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  float frontDistance = getDistance();// 전방 거리 측정
  delay(500); 
  Serial.print("Distance : ");
  Serial.println(frontDistance);

  if (frontDistance > 15){
    moveForward();
  }
  else {
    stop();
    Serial.println("detected obstacle");
    myservo.write(0); // 초음파센서 좌측으로 회전
    delay(1000);
    float leftDistance = getDistance(); //leftDistance에 값 저장
    Serial.print("Left Distance : ");
    Serial.println(leftDistance);

    myservo.write(180); // 초음파센서 우측으로 회전
    delay(1000);
    float rightDistance = getDistance(); //rightDistance에 값 저장
    Serial.print("Right Distance : ");
    Serial.println(rightDistance);

    myservo.write(90); // 초음파 센서 원위치

    if(leftDistance > rightDistance && leftDistance > 15){
      turnLeft();//왼쪽으로 방향 전환
      delay(TURN_TIME); // 실제 회전시간에 맞춰 조절
    }
    else if(rightDistance > leftDistance && rightDistance > 15){
      turnRight();//오른쪽으로 방향 전환
      delay(TURN_TIME); // 실제 회전시간에 맞춰 조절
    }
    else{
      moveBackward();
      backdistance = getBackDistance();
      Serial.print("Back Distance : ");
      Serial.println(backdistance);
      if(backdistance < 15){
        stop();
      }
    }
    
  }
  delay(100);

}