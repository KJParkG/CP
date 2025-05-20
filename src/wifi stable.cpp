//초음파 센서를 통한 장애물 회피 로봇 및 Telnet 로깅 코드

// 2025 05 19 전방 다 막혔을때 로직 수정 (후진 뒤 180도 회전) -> (후진 뒤 좌우 탐색)
// 2025 05 20 회전 각도 줄이고 이상한 값 나올 시 다른 차량 몸체 회전하기

#include <Arduino.h>
#include "ESP32Servo.h"
#include <WiFi.h>

const char* ssid = "AtoZ_LAB";
const char* password = "atoz9897!";

// Telnet server (from Code 2)
const uint16_t telnetPort = 23; // Telnet 기본 포트
WiFiServer telnetServer(telnetPort);
WiFiClient telnetClient; // 현재 연결된 클라이언트를 저장할 변수

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
  printlnLog(String(value));
}

void printlnLog(int value) {
  printlnLog(String(value));
}

#define SERVO_PIN 39
Servo myservo;

float backdistance; // 후방 거리

#define trigPin 42
#define echoPin 41
//#define backTrigPin 6
//#define backEchoPin 7

#define TURN_TIME 3000 // 실제 회전시간에 맞게 조절
#define SAFE_DISTANCE 20
#define MAX_DISTANCE 399
#define MIN_DISTANCE 3
// Motor driver pins (from Code 1)
//int ena = 3;
int in1 = 15;  //IN1
int in2 = 16; //IN2
int in3 = 17; //IN3
int in4 = 18; //IN4
//int enb = 18;

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
  distance = ((float)(340 * duration) / 10000) / 2; // 거리 = (음속 340 m/s * 시간(us)) / 2 / 10000 → cm로 변환
  return distance;

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
    float sample[5];
    int validSampleCount = 0;
    for (int i=0; i<5; i++){
        float currentDistance = measureDistance(trigPin, echoPin);
        
        if (currentDistance >= MIN_DISTANCE && currentDistance <= MAX_DISTANCE){
            sample[validSampleCount++] = currentDistance;
        }
        delay(20);
    }
    if (validSampleCount == 0 || validSampleCount > MAX_DISTANCE || validSampleCount  < MIN_DISTANCE){
        return MAX_DISTANCE + 1;
    }
    std::sort(sample, sample + validSampleCount);
    float medianDistance = sample[validSampleCount / 2];
    return medianDistance;
}

float getDistance(){
  return getFilterDistance();
}

// float getBackDistance(){ // from Code 1 (commented out section)
//   return measureDistance(backTrigPin, backEchoPin);
// }

// Motor control functions (from Code 1, modified to use printlnLog)
void moveForward(){
  //analogWrite(ena, 255); // from Code 1
  //analogWrite(enb,  255); // from Code 1
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  printlnLog("Go Forward"); // Modified for Telnet
}

void moveBackward(){
  //analogWrite(ena, 255); // from Code 1
  //analogWrite(enb,  255); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  printlnLog("Go Back"); // Modified for Telnet
  delay(3000); // from Code 1
}

void stop(){
  //analogWrite(ena, 0); // from Code 1
  //analogWrite(enb,  0); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  printlnLog("Stop"); // Modified for Telnet
  delay(500); // from Code 1
}

void turnLeft(){
  //analogWrite(ena, 255); // from Code 1
  //analogWrite(enb,  255); // from Code 1
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  printlnLog("Turn Left"); // Modified for Telnet
  delay(TURN_TIME); // 실제 회전시간에 맞춰 조절
}

void turnRight(){
  //analogWrite(ena, 255); // from Code 1
  //analogWrite(enb,  255); // from Code 1
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  printlnLog("Turn Right"); // Modified for Telnet
  delay(TURN_TIME); // 실제 회전시간에 맞춰 조절
}

void setup() {
  myservo.attach(SERVO_PIN);
  Serial.begin(115200); // Initialize Serial communication
  delay(1000); // Wait for Serial to initialize

  // WiFi Connection (from Code 2)
  printlnLog(" "); // Use printlnLog for initial messages too
  printLog("Connecting to ");
  printlnLog(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (telnetClient && telnetClient.connected()) {
        telnetClient.print(".");
    }
  }

  printlnLog("");
  printlnLog("WiFi connected.");
  printLog("IP address: ");
  printlnLog(WiFi.localIP().toString()); // 이 IP 주소를 Telnet 접속 시 사용합니다.

  // Telnet Server Start (from Code 2)
  telnetServer.begin();
  telnetServer.setNoDelay(true); // 데이터 즉시 전송 설정
  printLog("Telnet server started on port ");
  printlnLog(telnetPort);
  printlnLog("You can connect using a Telnet client (e.g., PuTTY).");

  // PinModes for sensors and motors (from Code 1)
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //pinMode(backTrigPin, OUTPUT); // from Code 1
  //pinMode(backEchoPin, INPUT);  // from Code 1
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //pinMode(ena, OUTPUT); // from Code 1
  //pinMode(enb, OUTPUT); // from Code 1

  myservo.write(90); // Center servo
  delay(500);
}

void loop() {
  if (telnetServer.hasClient()) {
    if (telnetClient && telnetClient.connected()) {
      telnetClient.stop();
      printlnLog("Previous Telnet client disconnected.");
    }
    telnetClient = telnetServer.available();
    if (telnetClient) {
      printLog("New Telnet client connected: ");
      printlnLog(telnetClient.remoteIP().toString());
      telnetClient.println("Welcome to ESP32 Remote Monitor!");
      telnetClient.flush();
    }
  }

  float frontDistance = getDistance(); // 전방 거리 측정
  delay(100);
  printLog("Distance : ");
  printlnLog(frontDistance);
  Serial.println(WiFi.localIP().toString());

  if (frontDistance > SAFE_DISTANCE && frontDistance < MAX_DISTANCE && frontDistance > MIN_DISTANCE){
    moveForward();
  }

  else {
    stop();
    printlnLog("detected obstacle");
    while(true) { // 길을 찾을 때까지 반복
      printlnLog("finding path...");
      myservo.write(15);
      delay(800);
      float leftDistance = getDistance();
      printLog("Left Distance: ");
      printlnLog(leftDistance);

      myservo.write(165);
      delay(1000);
      float rightDistance = getDistance();
      printLog("Right Distance: ");
      printlnLog(rightDistance);

      myservo.write(90);
      delay(500);

      if (leftDistance > SAFE_DISTANCE && leftDistance > rightDistance) {
        turnLeft();
        break;
      }
      else if (rightDistance > SAFE_DISTANCE && rightDistance > leftDistance) {
        turnRight();
        break;
      }
      else {
        printlnLog("Path unclear, moving back and rescanning...");
        stop();
        moveBackward();
        stop();
        
      }
    }
  }

  // Periodic Telnet uptime message (from Code 2)
  static unsigned long lastMsgTime = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - lastMsgTime > 1000) { // Send uptime every 1 second
    lastMsgTime = currentMillis;
    printLog("Uptime: ");
    printlnLog(String(currentMillis / 1000) + " seconds");
  }

  delay(100); // Main loop delay (from Code 1)
}