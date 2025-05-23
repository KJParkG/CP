#include <Arduino.h>
#include "ESP32Servo.h" // 서보 모터 사용
#include <WiFi.h>       // WiFi 및 Telnet 사용        // std::fabs 사용 위함

// --- WiFi 설정 ---
const char* ssid = "AtoZ_LAB";        // 여기에 실제 WiFi SSID를 입력하세요.
const char* password = "atoz9897!"; // 여기에 실제 WiFi 비밀번호를 입력하세요.

// --- Telnet 서버 설정 ---
const uint16_t telnetPort = 23;
WiFiServer telnetServer(telnetPort);
WiFiClient telnetClient;

// --- 로깅 함수 (Serial + Telnet) ---
void printLog(const String &message) {
  Serial.print(message);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(message);
  }
}

void printlnLog(const String &message) {
  Serial.println(message);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(message);
  }
}

void printlnLog(float value) {
  // 부동소수점 값을 문자열로 변환하여 로깅 (소수점 2자리)
  char buffer[16]; // 충분한 크기의 버퍼
  snprintf(buffer, sizeof(buffer), "%.2f", value);
  printlnLog(String(buffer));
}

void printlnLog(int value) {
  printlnLog(String(value));
}


// --- 전역 상수 및 설정 ---
// 초음파 센서 핀 (HC-SR04 기준)
const int TRIG_PIN = 42;
const int ECHO_PIN = 41;

// 거리 측정 관련 설정
const float MAX_DISTANCE = 400.0f; // cm 단위, 센서가 안정적으로 측정 가능한 최대 거리 또는 인식할 최대 유효 거리
const float MIN_DISTANCE = 3.0f;   // cm 단위, 센서가 안정적으로 측정 가능한 최소 거리

// 로봇 행동 관련 설정
const float SAFE_DISTANCE = 20.0f; // cm 단위, 이 거리보다 가까우면 장애물로 인식
const int TURN_TIME = 4000;        // ms 단위, 회전 시간 (사용자 환경에 맞게 조절 필요, 기존 4000에서 줄여봄)
const int MOVE_BACK_TIME = 2000;   // ms 단위, 후진 시간
const int SERVO_PIN = 39;          // 서보 모터 연결 핀

// 모터 드라이버 핀 (L298N 기준)
int ena = 13; // 왼쪽 모터 속도 제어 (PWM)
int in1 = 15; // 왼쪽 모터 방향 제어 1
int in2 = 16; // 왼쪽 모터 방향 제어 2
int in3 = 17; // 오른쪽 모터 방향 제어 1
int in4 = 18; // 오른쪽 모터 방향 제어 2
int enb = 12; // 오른쪽 모터 속도 제어 (PWM)

// --- 전역 변수 ---
Servo myservo; // 서보 객체

// --- 센서 프로필 ID 정의 ---
enum SENSOR_PROFILE_ID {
  PROFILE_FRONT,
  PROFILE_LEFT,
  PROFILE_RIGHT,
  PROFILE_COUNT // 프로필 개수 (배열 크기 지정용)
};

// --- 실제 거리 측정 함수 (기본) ---
float measureDistanceBasic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // pulseIn의 타임아웃은 왕복 시간을 고려해야 하므로, MAX_DISTANCE에 도달하는 시간의 두 배 이상으로 설정
  // 예: 400cm -> 4m. 음속 343m/s. 4m / 343m/s * 2 (왕복) * 1,000,000 us/s = 23323 us
  long duration = pulseIn(echoPin, HIGH, (long)(MAX_DISTANCE / 34.3 * 2.0 * 1000.0) + 10000); // 여유 타임아웃
  float distance = duration * 0.0343 / 2.0;

  if (distance == 0 || distance > MAX_DISTANCE * 1.5) {
    return MAX_DISTANCE + 10.0f;
  }
  return distance;
}

// --- 필터링된 거리 측정 함수 (첫 번째 코드에서 가져옴) ---
float getFilterDistance(SENSOR_PROFILE_ID profileId) {
    static float lastStatefulDistances[PROFILE_COUNT];
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
        float currentDistance = measureDistanceBasic(TRIG_PIN, ECHO_PIN); // 전역 핀 사용
        readingsTaken++;

        if (currentDistance > MAX_DISTANCE) { // 최댓값이 측정된 횟수 구하기
            readingsAboveMax++;
        } else if (currentDistance >= MIN_DISTANCE) { // 유효한 범위 내에서 측정된 값이면
            if (validSampleCount == 0) { // 샘플이 없을 시 첫 배열에 샘플 넣음
                samples[validSampleCount++] = currentDistance;
            } else {
                // 이전 유효 샘플과 비교해서 오차가 작다면 넣음
                if (std::fabs(currentDistance - samples[validSampleCount - 1]) <= errorThreshold) {
                    samples[validSampleCount++] = currentDistance;
                }
                // 오차가 크면 해당 샘플은 버림 (필터링 강화)
            }
        }
        delay(25); // 안정적인 측정을 위한 짧은 딜레이 (기존 20ms)
        // 유효 샘플이 3개 이상 모이면 더 이상 측정하지 않고 평균 계산 (반응 속도 개선 목적)
        if (validSampleCount >= 3 && i >=2) { // 최소 3번은 측정 시도 후, 유효 샘플 3개 이상이면 break
             break;
        }
    }

    float resultDistance;

    if (validSampleCount > 0) { // 유효한 샘플이 하나라도 있다면
        float sum = 0;
        for (int i = 0; i < validSampleCount; i++) {
            sum += samples[i];
        }
        resultDistance = sum / validSampleCount;
        lastStatefulDistances[profileId] = resultDistance; // 마지막 유효 상태 업데이트
    } else { // 유효한 샘플이 하나도 없는 경우
        // 모든 시도가 MAX_DISTANCE를 초과했는지, 아니면 다른 이유로 유효 샘플이 없는지 확인
        if (readingsTaken > 0 && readingsAboveMax == readingsTaken) {
            if (lastStatefulDistances[profileId] >= MIN_DISTANCE && lastStatefulDistances[profileId] < MAX_DISTANCE) {
                 resultDistance = -1.0f; // 이전엔 괜찮았는데 지금 다 MAX면 오류로 보자
                 lastStatefulDistances[profileId] = -1.0f; // 상태도 오류로 업데이트
            } else {
                 resultDistance = MAX_DISTANCE; // 원래 MAX였거나 초기상태면 MAX로
                 lastStatefulDistances[profileId] = MAX_DISTANCE;
            }
        } else {
            // 그 외의 경우 (MIN_DISTANCE 미만 값, 간헐적 오류 등)는 측정 실패(-1.0f)로 처리
            resultDistance = -1.0f;
            lastStatefulDistances[profileId] = -1.0f; // 상태도 오류로 업데이트
        }
    }
    return resultDistance;
}


// --- 로봇 동작 함수 (첫 번째 코드 기반, 로깅 추가) ---
void moveForward() {
  analogWrite(ena, 200); // 속도 (0-255)
  analogWrite(enb, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  printlnLog("Go Forward");
}

void moveBackward(int time) {
  analogWrite(ena, 180); // 후진 속도
  analogWrite(enb, 180);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  printlnLog("Go Back");
  delay(time);
}

// 함수 이름을 stopMotors로 변경 (WiFiClient의 stop 메소드와 충돌 방지)
void stopMotors() {
  analogWrite(ena, 0);
  analogWrite(enb, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  printlnLog("Stop");
  // 원본 코드에는 stop() 후 delay(500)이 있었으나, 상황에 따라 조절. 여기서는 짧게.
  delay(200);
}

void turnLeft(int time) {
  analogWrite(ena, 200); // 회전 속도
  analogWrite(enb, 200);
  digitalWrite(in1, LOW);  // 왼쪽 바퀴 후진
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); // 오른쪽 바퀴 전진
  digitalWrite(in4, LOW);
  printlnLog("Turn Left");
  delay(time);
}

void turnRight(int time) {
  analogWrite(ena, 200); // 회전 속도
  analogWrite(enb, 200);
  digitalWrite(in1, HIGH); // 왼쪽 바퀴 전진
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  // 오른쪽 바퀴 후진
  digitalWrite(in4, HIGH);
  printlnLog("Turn Right");
  delay(time);
}

// --- 아두이노 표준 함수 ---
void setup() {
  Serial.begin(115200);
  delay(1000); // 시리얼 안정화 대기

  printlnLog(" ");
  printLog("Robot Starting... Trying to connect to WiFi: ");
  printlnLog(ssid);

  WiFi.begin(ssid, password);
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retries < 20) { // 최대 10초간 시도
    delay(500);
    Serial.print("."); // WiFi 연결 중에는 Telnet이 안되므로 Serial로만 출력
    wifi_retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    printlnLog(""); // 연결 성공 후에는 printlnLog 사용 가능
    printlnLog("WiFi connected.");
    printLog("IP address: ");
    printlnLog(WiFi.localIP().toString());

    telnetServer.begin();
    telnetServer.setNoDelay(true);
    printLog("Telnet server started on port ");
    printlnLog(telnetPort);
    printlnLog("You can connect using a Telnet client.");
  } else {
    printlnLog("");
    printlnLog("Failed to connect to WiFi. Running in offline mode (Serial log only).");
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  myservo.attach(SERVO_PIN);
  myservo.write(90); // 서보 모터 초기 위치 (정면)
  delay(500);        // 서보 안정화 시간

  printlnLog("Robot Setup Complete. Starting main loop...");
  stopMotors(); // 시작 시 정지 상태
}

void loop() {
  // Telnet 클라이언트 연결 관리
  if (WiFi.status() == WL_CONNECTED && telnetServer.hasClient()) {
    if (telnetClient && telnetClient.connected()) {
      telnetClient.stop(); // WiFiClient의 stop()
      printlnLog("Previous Telnet client disconnected.");
    }
    telnetClient = telnetServer.available();
    if (telnetClient && telnetClient.connected()) {
      printLog("New Telnet client connected: ");
      printlnLog(telnetClient.remoteIP().toString());
      telnetClient.println("--- Welcome to ESP32 Robot Monitor ---");
      telnetClient.flush();
    }
  }

  float frontDistance = getFilterDistance(PROFILE_FRONT);
  printLog("Front Distance: ");
  printlnLog(frontDistance);

  if (frontDistance > SAFE_DISTANCE) {
    moveForward();
    printlnLog(WiFi.localIP().toString());
  } else {
    stopMotors(); // 장애물 감지 또는 오류 시 일단 정지
    if (frontDistance == -1.0f) {
      printlnLog("Error reading front distance. Initiating pathfinding...");
    } else {
      printLog("Obstacle detected at front (");
      printLog(String(frontDistance, 2)); // 소수점 2자리
      printlnLog(" cm). Initiating pathfinding...");
    }

    // --- 첫 번째 코드의 경로 탐색 로직 시작 ---
    while (true) {
      printlnLog("Finding path logic: Scanning sides...");
      myservo.write(15); // 서보 좌측으로 (약 75도 좌향)
      delay(800);        // 서보 이동 및 안정화 시간 (첫 번째 코드 값)
      float leftDistance = getFilterDistance(PROFILE_LEFT);
      printLog("  Left Distance: ");
      printlnLog(leftDistance);

      myservo.write(165); // 서보 우측으로 (약 75도 우향)
      delay(1000);        // 서보 이동 및 안정화 시간 (첫 번째 코드 값)
      float rightDistance = getFilterDistance(PROFILE_RIGHT);
      printLog("  Right Distance: ");
      printlnLog(rightDistance);

      myservo.write(90); // 서보 중앙으로 복귀
      delay(500);        // 서보 이동 및 안정화 시간 (첫 번째 코드 값)

      // 첫 번째 코드의 조건절을 최대한 따름

      bool leftPathValid = (leftDistance != -1.0f && leftDistance > SAFE_DISTANCE);
      bool rightPathValid = (rightDistance != -1.0f && rightDistance > SAFE_DISTANCE);

      if (leftPathValid && (!rightPathValid || leftDistance >= rightDistance)) {
        // 좌측 경로가 유효하고, (우측 경로가 유효하지 않거나, 유효하다면 좌측이 더 넓거나 같을 때)
        printlnLog("Path found: Turning Left.");
        turnLeft(TURN_TIME);
        // stopMotors(); // 원본에는 회전 후 명시적 stop이 없었음. 필요시 추가.
        break; // while 루프 탈출
      } else if (rightPathValid) {
        // 우측 경로가 유효하고, (위 조건에서 걸리지 않았으므로, 좌측이 유효하지 않거나 우측이 더 넓을 때)
        printlnLog("Path found: Turning Right.");
        turnRight(TURN_TIME);
        // stopMotors(); // 원본에는 회전 후 명시적 stop이 없었음. 필요시 추가.
        break; // while 루프 탈출
      } else {
        // 양쪽 다 길이 없거나 오류인 경우
        printlnLog("Path unclear or both sides blocked/error. Moving back and rescanning...");
        // stopMotors(); // 이미 위에서 호출됨, 또는 moveBackward 전에 호출될 수 있음
        moveBackward(MOVE_BACK_TIME);
        stopMotors(); // 후진 후에는 반드시 정지 (원본 코드 로직)
        // 루프 계속 (다시 스캔)
      }
    }
    // --- 첫 번째 코드의 경로 탐색 로직 끝 ---
  }

  // 주기적인 Telnet 메시지 (예: 업타임) - WiFi 연결 시에만
  static unsigned long lastTelnetMsgTime = 0;
  unsigned long currentMillis = millis();
  if (WiFi.status() == WL_CONNECTED && telnetClient && telnetClient.connected() && (currentMillis - lastTelnetMsgTime > 5000)) { // 5초마다
    lastTelnetMsgTime = currentMillis;
    // telnetClient.print("Uptime: "); // 로그가 너무 많아질 수 있어 선택적 활성화
    // telnetClient.print(String(currentMillis / 1000));
    // telnetClient.println(" seconds");
  }

  delay(100); // 메인 루프의 짧은 딜레이 (첫 번째 코드 값)
}