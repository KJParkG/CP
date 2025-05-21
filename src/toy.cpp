//초음파 센서를 통한 장애물 회피 로봇 코드

// 2025 05 19 전방 다 막혔을때 로직 수정 (후진 뒤 180도 회전) -> (후진 뒤 좌우 탐색)
// 2025 05 20 초음파센서 튀는 값 나올 시 장애물 감지로 판단 ->> 초음파센서 튀는 값 어떻게 잡을건지??
// 2025 05 21 오차 발생시 다르게 리턴하는 로직 수정

// 해야할 것 이상한 값 해결하는 로직 -> 오차 발생시

#include <Arduino.h>
#include "ESP32Servo.h"

#define SERVO_PIN 39
Servo myservo;

bool justTurned = false;

#define trigPin 42
#define echoPin 41
//#define backTrigPin 6
//#define backEchoPin 7

#define TURN_TIME 4000 // 실제 회전시간에 맞게 조절
#define SAFE_DISTANCE 20
#define MAX_DISTANCE 399
#define MIN_DISTANCE 3
// Motor driver pins (from Code 1)
int ena = 13;
int in1 = 15;  //IN1
int in2 = 16; //IN2
int in3 = 17; //IN3
int in4 = 18; //IN4
int enb = 12;

// 초음파센서 거리 측정 함수 (from Code 1 / Code 2, functionally identical)
float measureDistance(int trig, int echo){
  float duration, distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH, 30000); // 타임아웃 30ms
    if (duration == 0){
        return 0;
    }
    else{
      distance = ((float)(340 * duration) / 10000) / 2; // 거리 = (음속 340 m/s * 시간(us)) / 2 / 10000 → cm로 변환
      return distance;
    }
  // 5번 실행하여 평균 구하는 함수 // 안정성 ↑ 반응속도 ↓
  // float total = 0;
  // int count = 3; // 실행 할 횟수
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

float getFilterDistance(){
  const int count = 7;
  float sample[count];
  int validSampleCount = 0;

  const float error = 5.0;

   for (int i=0; i<count; i++){
       float currentDistance = measureDistance(trigPin, echoPin);
      if (currentDistance > MAX_DISTANCE || currentDistance < MIN_DISTANCE){
        return MAX_DISTANCE;
      }
        if (currentDistance >= MIN_DISTANCE && currentDistance <= MAX_DISTANCE){
          if(validSampleCount == 0 || fabs(currentDistance - sample[validSampleCount-i] <= error))
            sample[validSampleCount++] = currentDistance;
          if(validSampleCount != 0 && (currentDistance > MAX_DISTANCE || currentDistance < MIN_DISTANCE)){
            return -1;
          }
         }
        delay(20);
        }

    if (validSampleCount > 0){
      float sum = 0;
      for (int i=0; i<validSampleCount; i++){
        sum += sample[i];
      }
      return sum / validSampleCount;
    }
    else{
      return -1;
    }
}

       

void moveForward(){
  analogWrite(ena, 250); // from Code 1
  analogWrite(enb, 255); // from Code 1
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Go Forward"); // Modified for Telnet
}

void moveBackward(int time){
  analogWrite(ena, 250); // from Code 1
  analogWrite(enb,  255); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Go Back"); // Modified for Telnet
  delay(time); // from Code 1
}

void stop(){
  analogWrite(ena, 0); // from Code 1
  analogWrite(enb,  0); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  Serial.println("Stop"); // Modified for Telnet
  delay(500); // from Code 1
}

void turnLeft(int time){
  analogWrite(ena, 250); // from Code 1
  analogWrite(enb,  255); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Turn Left"); // Modified for Telnet
  delay(time); // 실제 회전시간에 맞춰 조절
}

void turnRight(int time){
  analogWrite(ena, 250); // from Code 1
  analogWrite(enb,  255); // from Code 1
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Turn Right"); // Modified for Telnet
  delay(time); // 실제 회전시간에 맞춰 조절
}

void setup() {
  myservo.attach(SERVO_PIN);
  Serial.begin(115200); // Initialize Serial communication
  delay(1000); // Wait for Serial to initialize

  // PinModes for sensors and motors (from Code 1)
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //pinMode(backTrigPin, OUTPUT); // from Code 1
  //pinMode(backEchoPin, INPUT);  // from Code 1
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT); // from Code 1
  pinMode(enb, OUTPUT); // from Code 1

  myservo.write(90); // Center servo
  delay(500);
}

void loop() {

  float frontDistance = getFilterDistance();// 전방 거리 측정

  if (frontDistance > SAFE_DISTANCE){
    moveForward();
  }

  else {
    stop();
    Serial.println("detected obstacle");
    while(true) { // 길을 찾을 때까지 반복
      Serial.println("finding path...");
      myservo.write(5);
      delay(800);
      float leftDistance = getFilterDistance();
      Serial.print("Left Distance: ");
      Serial.println(leftDistance);

      myservo.write(175);
      delay(1000);
      float rightDistance = getFilterDistance();
      Serial.print("Right Distance: ");
      Serial.println(rightDistance);

      myservo.write(90);
      delay(500);

      if (leftDistance > SAFE_DISTANCE && leftDistance >= rightDistance) {
        turnLeft(TURN_TIME);
        break;
      }
      else if (rightDistance > SAFE_DISTANCE && rightDistance >= leftDistance) {
        turnRight(TURN_TIME);
        break;
      }
      else {
        Serial.println("Path unclear, moving back and rescanning...");
        stop();
        moveBackward(2500);
        stop();
        
      }
    }
  }

  delay(100); // Main loop delay (from Code 1)
}