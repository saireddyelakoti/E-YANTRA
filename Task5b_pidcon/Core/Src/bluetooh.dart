#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Motor pins
int m1 = 19, m2 = 18, m3 = 4, m4 = 5;

// Ultrasonic pins
#define TRIG_PIN 13
#define ECHO_PIN 12

// Movement tracking
bool training = false, replaying = false;

char currentPath;

char pathA[100], pathB[100];

unsigned int durationA[100], durationB[100];

int stepCountA = 0, stepCountB = 0;

void setup() {

  Serial.begin(115200);

  SerialBT.begin("ESP32_Robot");

  Serial.println("Bluetooth started. Waiting for connections...");

  pinMode(m1, OUTPUT); pinMode(m2, OUTPUT);

  pinMode(m3, OUTPUT); pinMode(m4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);

  pinMode(ECHO_PIN, INPUT);

  stopRobot();

  Serial.println("Robot Ready!");

}

void loop() {

  if (SerialBT.available()) {

    char command = SerialBT.read();

    SerialBT.print("Received: "); SerialBT.println(command);

    switch (command) {

      case 'A': startTraining('A'); break;

      case 'B': startTraining('B'); break;

      case 'X': startReplaying('A'); break;

      case 'Y': startReplaying('B'); break;

      case '1': case '2': case '3': case '4': case '5': moveRobot(command); break;

      default: SerialBT.println("Invalid Command!"); break;

    }

  }

}

void startTraining(char path) {

  SerialBT.print("Training Path: "); SerialBT.println(path);

  training = true; replaying = false; currentPath = path;

  if (path == 'A') stepCountA = 0; else stepCountB = 0;

}

void startReplaying(char path) {

  SerialBT.print("Replaying Path: "); SerialBT.println(path);

  training = false; replaying = true; currentPath = path;

  replayMovement(path);

  rotate180();

  returnToStart(path);

}

void moveRobot(char command) {

  static char lastCommand = '0';

  static unsigned long lastTime = 0;

  unsigned long currentTime = millis();

  if (command != lastCommand && lastCommand != '0') {

    unsigned long duration = currentTime - lastTime;

    SerialBT.print("Movement Duration: "); SerialBT.println(duration);

    if (training) {

      if (currentPath == 'A') {

        pathA[stepCountA] = lastCommand;

        durationA[stepCountA++] = duration;

      } else {

        pathB[stepCountB] = lastCommand;

        durationB[stepCountB++] = duration;

      }

    }

  }

  lastCommand = command;

  lastTime = millis();

  switch (command) {

    case '1': front(); break;

    case '2': back(); break;

    case '3': left(); break;

    case '4': right(); break;

    case '5': stopRobot(); break;

  }

}

void replayMovement(char path) {

  int count = (path == 'A') ? stepCountA : stepCountB;

  char* commands = (path == 'A') ? pathA : pathB;

  unsigned int* durations = (path == 'A') ? durationA : durationB;

  for (int i = 0; i < count; i++) {

    safeMoveWithUltrasonic(commands[i], durations[i]);

  }

  stopRobot();

  SerialBT.println("Replay Complete!");

}

void safeMoveWithUltrasonic(char command, unsigned int duration) {

  moveRobot(command);

  unsigned long startTime = millis();

  unsigned long elapsed = 0;

  while (elapsed < duration) {

    if (getDistance() < 5.0) {

      SerialBT.println("Obstacle detected! Pausing...");

      stopRobot();

      while (getDistance() < 5.0) {

        delay(100); // Wait for obstacle to clear

      }

      SerialBT.println("Obstacle cleared. Resuming...");

      moveRobot(command);

    }

    delay(10); // Small delay to avoid busy loop

    elapsed = millis() - startTime;

  }

  stopRobot();

}

void rotate180() {

  SerialBT.println("Waiting before rotation...");

  delay(5000);

  SerialBT.println("Rotating 180 degrees...");

  left();

  delay(2900); // Adjust as per robot speed

  stopRobot();

  SerialBT.println("Rotation Complete!");

}

void returnToStart(char path) {

  SerialBT.println("Returning to Start Position...");

  int count = (path == 'A') ? stepCountA : stepCountB;

  char* commands = (path == 'A') ? pathA : pathB;

  unsigned int* durations = (path == 'A') ? durationA : durationB;

  for (int i = count - 1; i >= 0; i--) {

    char reverseCommand = getReverseCommand(commands[i]);

    safeMoveWithUltrasonic(reverseCommand, durations[i]);

  }

  stopRobot();

  SerialBT.println("Returned to Start!");

}

char getReverseCommand(char command) {

  switch (command) {

    case '1': return '1'; // Forward remains Forward

    case '2': return '2'; // Back remains Back

    case '3': return '4'; // Left becomes Right

    case '4': return '3'; // Right becomes Left

    default: return '5';  // Stop

  }

}

// Movement Functions

void front() { move(1, 0, 1, 0); }

void back()  { move(0, 1, 0, 1); }

void left()  { move(1, 0, 0, 1); }

void right() { move(0, 1, 1, 0); }

void stopRobot() { move(0, 0, 0, 0); }

void move(int m1s, int m2s, int m3s, int m4s) {

  digitalWrite(m1, m1s); digitalWrite(m2, m2s);

  digitalWrite(m3, m3s); digitalWrite(m4, m4s);

}

// Ultrasonic Distance Function

float getDistance() {

  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);

  float distance = duration * 0.034 / 2;

  return distance;

}