//초음파 센서 모터 제어 코드
#include <Arduino.h>
#include "ESP32Servo.h"

#define SERVO_PIN 39

Servo myservo;

float backdistance; // 후방 거리

#define trigPin 42
#define echoPin 41
//#define backTrigPin 6
//#define backEchoPin 7

#define TURN_TIME 4000 // 실제 회전시간에 맞게 조절

//int ena = 3;
int in1 = 15;  //IN1
int in2 = 16; //IN2

int in3 = 17; //IN3
int in4 = 18; //IN4
//int enb = 18;

float measureDistance(int trig, int echo){
  // 초음파센서 거리 측정 함수
  float duration, distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH, 30000);
  distance = ((float)(340*duration) / 10000) / 2; // 거리 = (음속 340 m/s * 시간(us)) / 2 / 10000 → cm로 변환
  delay(10);
  return distance;

  // // 5번 실행하여 평균 구하는 함수 // 안정성 ↑ 반응속도 ↓
  // float total = 0;
  // int count = 3;
  // for (int i = 0; i < count; i++) {
  //   digitalWrite(trig, LOW);
  //   delayMicroseconds(2);
  //   digitalWrite(trig, HIGH);
  //   delayMicroseconds(10);
  //   digitalWrite(trig, LOW);
  //   float duration = pulseIn(echo, HIGH, 30000);
  //   total += duration * 0.0343 / 2.0;
  //   delay(10);  // 너무 빠른 반복 방지
  // }
  // return total / count;
}
float getDistance(){
  return measureDistance(trigPin, echoPin);
}

// float getBackDistance(){
//   return measureDistance(backTrigPin, backEchoPin);
// }
// put function declarations here:
void moveForward(){
  //analogWrite(ena, 255);
  //analogWrite(enb,  255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Go Forward");
}

void moveBackward(){
  //analogWrite(ena, 255);
  //analogWrite(enb,  255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Go Back");
  delay(4000);
}

void stop(){
  //analogWrite(ena, 0);
  //analogWrite(enb,  0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  Serial.println("Stop");
  delay(500);
}

void turnLeft(){
  //analogWrite(ena, 255);
  //analogWrite(enb,  255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Turn Left");
  delay(TURN_TIME); // 실제 회전시간에 맞춰 조절
  //좌회전
}

void turnRight(){
  //analogWrite(ena, 255);
  //analogWrite(enb,  255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Turn Right");
  delay(TURN_TIME); // 실제 회전시간에 맞춰 조절
  //우회전
}

void setup() {
  // put your setup code here, to run once:
  myservo.attach(SERVO_PIN);
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //pinMode(backTrigPin, OUTPUT);
  //pinMode(backEchoPin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //pinMode(ena, OUTPUT);
  //pinMode(enb, OUTPUT);

  myservo.write(90);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  float frontDistance = getDistance();// 전방 거리 측정
  delay(100);
  Serial.print("Distance : ");
  Serial.println(frontDistance);

  if (frontDistance > 20){
    moveForward();
  }
  else {
    stop();
    Serial.println("detected obstacle");
    myservo.write(15); // 초음파센서 좌측으로 회전
    delay(800); // 대략적인 서보모터 회전 시간
    float leftDistance = getDistance(); //leftDistance에 값 저장
    Serial.print("Left Distance : ");
    Serial.println(leftDistance);

    myservo.write(165); // 초음파센서 우측으로 회전
    delay(1000); // 대략적인 서보모터 회전 시간
    float rightDistance = getDistance(); //rightDistance에 값 저장
    Serial.print("Right Distance : ");
    Serial.println(rightDistance);

    myservo.write(90); // 초음파 센서 원위치
    delay(500);

    if(leftDistance > rightDistance && leftDistance > 20){
      turnLeft();//왼쪽으로 방향 전환
    }
    else if(rightDistance > leftDistance && rightDistance > 20){
      turnRight();//오른쪽으로 방향 전환
    }
    else{
      // moveBackward();
      // backdistance = getBackDistance();
      // Serial.print("Back Distance : ");
      // Serial.println(backdistance);
      // if(backdistance < 15 && backdistance > 0){
      //   stop();
      //   // 정지 후 다시 전진하도록 하는게 아닌 새롭게 길찾는 알고리즘을 하나 만들어야 함.
      //   // 제자리 회전을 해서 새로 길을 찾거나 하는 코드.
      //   // Escape code
      // }
      stop();
      moveBackward();
      turnLeft();
      turnLeft(); // 좌측 회전 2번을 통해 반바퀴 회전
    }
  }
  delay(100);

}

