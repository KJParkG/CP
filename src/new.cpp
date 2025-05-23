
#include "ESP32Servo.h" // 서보 모터 사용
#include <Arduino.h>
// 2025 05 19 전방 다 막혔을때 로직 수정 (후진 뒤 180도 회전) -> (후진 뒤 좌우 탐색)
// 2025 05 20 초음파센서 튀는 값 나올 시 장애물 감지로 판단 ->> 초음파센서 튀는 값 어떻게 잡을건지??
// 2025 05 21 오차 발생시 다르게 리턴하는 로직 수정
// 2025 05 23 전방 이상값 발생시 오류 리턴

// --- 전역 상수 및 설정 ---
// 초음파 센서 핀 (HC-SR04 기준)
const int TRIG_PIN = 42;
const int ECHO_PIN = 41;

// 거리 측정 관련 설정
const float MAX_DISTANCE = 400.0f; // cm 단위, 센서가 안정적으로 측정 가능한 최대 거리 또는 인식할 최대 유효 거리
const float MIN_DISTANCE = 3.0f;   // cm 단위, 센서가 안정적으로 측정 가능한 최소 거리

// 로봇 행동 관련 설정
const float SAFE_DISTANCE = 20.0f; // cm 단위, 이 거리보다 가까우면 장애물로 인식
const int TURN_TIME = 4000;         // ms 단위, 회전 시간
const int MOVE_BACK_TIME = 1500;   // ms 단위, 후진 시간
const int SERVO_PIN = 39;          // 서보 모터 연결 핀

int ena = 13;
int in1 = 15;  //IN1
int in2 = 16; //IN2
int in3 = 17; //IN3
int in4 = 18; //IN4
int enb = 12;

// --- 전역 변수 ---
Servo myservo; // 서보 객체

// --- 센서 프로필 ID 정의 ---
enum SENSOR_PROFILE_ID {
  PROFILE_FRONT,
  PROFILE_LEFT,
  PROFILE_RIGHT,
  PROFILE_COUNT // 프로필 개수 (배열 크기 지정용)
};

// --- 실제 거리 측정 함수 (플레이스홀더) ---
// 사용하시는 초음파 센서 라이브러리나 직접 구현한 코드로 대체해야 합니다.
float measureDistance(int trigPin, int echoPin) {
  // HC-SR04 예시 로직 (실제 값으로 보정 필요)
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  if (distance == 0 || distance > MAX_DISTANCE * 1.5) { // 측정 실패 또는 너무 먼 값은 MAX_DISTANCE + 약간의 여유로 처리
    return MAX_DISTANCE + 10.0f; // MAX_DISTANCE보다 큰 값으로 반환하여 필터링 로직에서 처리되도록 함
  }
  return distance;
}

// --- 필터링된 거리 측정 함수 ---
float getFilterDistance(SENSOR_PROFILE_ID profileId) {
    static float lastStatefulDistances[PROFILE_COUNT]; // 각 방향 별 마지막 거리 상태 저장
    static bool isInitialized = false; 

    if (!isInitialized) {
        for (int i = 0; i < PROFILE_COUNT; ++i) {
            lastStatefulDistances[i] = -1.0f; // 처음에 호출 시 -1로 초기화
        }
        isInitialized = true;
    }

    const int samplesToAttempt = 7; // 측정 횟수
    float samples[samplesToAttempt];
    int validSampleCount = 0;
    const float errorThreshold = 5.0f; // 이전 유효 샘플과의 최대 허용 오차
    int readingsTaken = 0;
    int readingsAboveMax = 0;

    for (int i = 0; i < samplesToAttempt; i++) {
        float currentDistance = measureDistance(TRIG_PIN, ECHO_PIN); // 전역 핀 사용
        readingsTaken++;

        if (currentDistance > MAX_DISTANCE) { // 최댓값이 측정된 횟수 구하기
            readingsAboveMax++;
        } else if (currentDistance >= MIN_DISTANCE) { // 유효한 범위 내에서 측정된 값이면
            if (validSampleCount == 0) { // 샘플이 없을 시 첫 배열에 샘플 넣음
                samples[validSampleCount++] = currentDistance;
            } else {
                if (std::fabs(currentDistance - samples[validSampleCount - 1]) <= errorThreshold) { // 이전 값과 비교해서 오차가 작다면 넣음
                    samples[validSampleCount++] = currentDistance;
                }
            }
        }
        delay(20); // 안정적인 측정을 위한 짧은 딜레이
        if (validSampleCount == samplesToAttempt) {
            break;
        }
    }

    float resultDistance;

    if (validSampleCount > 0) {
        float sum = 0;
        for (int i = 0; i < validSampleCount; i++) {
            sum += samples[i];
        }
        resultDistance = sum / validSampleCount;
        lastStatefulDistances[profileId] = resultDistance;
    } else {
        if (readingsTaken > 0 && readingsAboveMax == readingsTaken) {
            if (lastStatefulDistances[profileId] >= MIN_DISTANCE && lastStatefulDistances[profileId] < MAX_DISTANCE) {
                resultDistance = -1.0f;
                lastStatefulDistances[profileId] = -1.0f;
            } else {
                resultDistance = MAX_DISTANCE;
                lastStatefulDistances[profileId] = MAX_DISTANCE;
            }
        } else {
            resultDistance = -1.0f;
            lastStatefulDistances[profileId] = -1.0f;
        }
    }
    return resultDistance;
}

// --- 로봇 동작 함수 (플레이스홀더) ---
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
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  myservo.attach(SERVO_PIN); // 서보 모터 핀 연결
  myservo.write(90);         // 서보 모터 초기 위치 (정면)
  delay(500);

  // 모터 핀 초기화 등 기타 설정
  // 예: pinMode(MOTOR_A_FWD, OUTPUT);
}

void loop() {
  float frontDistance = getFilterDistance(PROFILE_FRONT);
  Serial.print("Front Distance: ");
  Serial.println(frontDistance);

  if (frontDistance > SAFE_DISTANCE) {
    moveForward();
  } else {
    stop();
    if (frontDistance == -1.0f) {
      Serial.println("Error reading front distance. Finding path...");
    } else {
      Serial.print("Obstacle detected");
    }

    while (true) {
      Serial.println("Finding path logic...");
      myservo.write(15); // 서보 좌측으로
      delay(800);       // 서보 이동 및 안정화 시간
      float leftDistance = getFilterDistance(PROFILE_LEFT);
      Serial.print("Left Distance: ");
      Serial.println(leftDistance);

      myservo.write(165); // 서보 우측으로
      delay(1000);      // 서보 이동 및 안정화 시간 (좌->우 이동이므로 좀 더 길게)
      float rightDistance = getFilterDistance(PROFILE_RIGHT);
      Serial.print("Right Distance: ");
      Serial.println(rightDistance);

      myservo.write(90); // 서보 중앙으로
      delay(500);       // 서보 이동 및 안정화 시간

      bool leftPathValid = (leftDistance != -1.0f && leftDistance > SAFE_DISTANCE);
      bool rightPathValid = (rightDistance != -1.0f && rightDistance > SAFE_DISTANCE);

      if (leftPathValid && (!rightPathValid || leftDistance >= rightDistance)) {
        turnLeft(TURN_TIME);
        break;
      } else if (rightPathValid) {
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