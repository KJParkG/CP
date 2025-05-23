
#include "ESP32Servo.h" // 서보 모터 사용
#include <Arduino.h>
// 2025 05 19 전방 다 막혔을때 로직 수정 (후진 뒤 180도 회전) -> (후진 뒤 좌우 탐색)
// 2025 05 20 초음파센서 튀는 값 나올 시 장애물 감지로 판단 ->> 초음파센서 튀는 값 어떻게 잡을건지??
// 2025 05 21 오차 발생시 다르게 리턴하는 로직 수정
// 2025 05 23 최소거리에서 안전거리 사이 일때만 리턴 나머지는 그냥 최대값 반환

// 반사가 안되는 것과, 전방이 비어있는 것 어떻게 구분을 할까???
// --- 전역 상수 및 설정 ---
// 초음파 센서 핀 (HC-SR04 기준)
const int TRIG_PIN = 42;
const int ECHO_PIN = 41;

// 거리 측정 관련 설정
const float MAX_DISTANCE = 377.0f; // cm 단위, 센서가 안정적으로 측정 가능한 최대 거리 또는 인식할 최대 유효 거리
const float MIN_DISTANCE = 4.0f;   // cm 단위, 센서가 안정적으로 측정 가능한 최소 거리

// 로봇 행동 관련 설정
const float SAFE_DISTANCE = 25.0f; // cm 단위, 이 거리보다 가까우면 장애물로 인식
const int TURN_TIME = 4000;         // ms 단위, 회전 시간
const int MOVE_BACK_TIME = 2000;   // ms 단위, 후진 시간
const int SERVO_PIN = 39;          // 서보 모터 연결 핀

int ena = 13;
int in1 = 15;  //IN1
int in2 = 16; //IN2
int in3 = 17; //IN3
int in4 = 18; //IN4
// int enb = 12;

// --- 전역 변수 ---
Servo myservo; // 서보 객체

// --- 센서 프로필 ID 정의 ---

// --- 실제 거리 측정 함수 (플레이스홀더) ---
// 사용하시는 초음파 센서 라이브러리나 직접 구현한 코드로 대체해야 합니다.
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  return duration * 0.034 / 2;
}

// --- 거리 필터 ---
float getFilteredDistance() {
  float sum = 0;
  int count = 0;
  for (int i = 0; i < 5; i++) {
    float d = measureDistance();
    if (d >= MIN_DISTANCE && d <= SAFE_DISTANCE) {
      sum += d;
      count++;
    }
    delay(30);
  }
  return (count > 0) ? 0 : measureDistance();
}

// --- 로봇 동작 함수 (플레이스홀더) ---
void moveForward(){
  analogWrite(ena, 230); // from Code 1
  //analogWrite(enb, 250); // from Code 1
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Go Forward"); // Modified for Telnet
}

void moveBackward(int time){
  analogWrite(ena, 230); // from Code 1
  //analogWrite(enb,  250); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Go Back"); // Modified for Telnet
  delay(time); // from Code 1
}

void stop(){
  //analogWrite(ena, 0); // from Code 1
  //analogWrite(enb,  0); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  Serial.println("Stop"); // Modified for Telnet
  delay(500); // from Code 1
}

void turnLeft(int time){
  analogWrite(ena, 255); // from Code 1
  //analogWrite(enb,  250); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Turn Left"); // Modified for Telnet
  delay(time); // 실제 회전시간에 맞춰 조절
}

void turnRight(int time){
  analogWrite(ena, 255); // from Code 1
  //analogWrite(enb,  250); // from Code 1
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Turn Right"); // Modified for Telnet
  delay(time); // 실제 회전시간에 맞춰 조절
}

// --- 아두이노 표준 함수 ---
void setup() {
  Serial.begin(9600); // 시리얼 통신 시작
  Serial.println("Robot Setup Complete. Starting loop...");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // pinMode(ena, OUTPUT);
  // pinMode(enb, OUTPUT);
  myservo.attach(SERVO_PIN); // 서보 모터 핀 연결
  myservo.write(90);         // 서보 모터 초기 위치 (정면)
  delay(500);

  // 모터 핀 초기화 등 기타 설정
  // 예: pinMode(MOTOR_A_FWD, OUTPUT);
}

void loop() {
  float frontDistance = getFilteredDistance();
  //Serial.print("Front Distance: ");
  //Serial.println(measureDistance(TRIG_PIN, ECHO_PIN));
  if (frontDistance != 0) {
    moveForward();
    Serial.print("Front Distance: ");
    Serial.println(frontDistance);
  } else {
    stop();
      Serial.print("Obstacle detected");

    while (true) {
      Serial.println("Finding path logic...");
      myservo.write(15); // 서보 좌측으로
      delay(800);       // 서보 이동 및 안정화 시간
      float leftDistance = measureDistance();
      Serial.print("Left Distance: ");
      Serial.println(leftDistance);

      myservo.write(165); // 서보 우측으로
      delay(1000);      // 서보 이동 및 안정화 시간 (좌->우 이동이므로 좀 더 길게)
      float rightDistance = measureDistance();
      Serial.print("Right Distance: ");
      Serial.println(rightDistance);

      myservo.write(90); // 서보 중앙으로
      delay(500);       // 서보 이동 및 안정화 시간

      if (leftDistance > SAFE_DISTANCE && leftDistance >= rightDistance) {
        turnLeft(TURN_TIME);
        break;
      } else if (rightDistance > SAFE_DISTANCE && rightDistance > leftDistance) {
        turnRight(TURN_TIME);
        break;
      } else {
        Serial.println("Path unclear, moving back and rescanning...");
        stop(); // 후진 전 정지
        moveBackward(MOVE_BACK_TIME);
        stop(); // 후진 후 정지
      }
    }
  }
  delay(100); // 메인 루프의 짧은 딜레이
}