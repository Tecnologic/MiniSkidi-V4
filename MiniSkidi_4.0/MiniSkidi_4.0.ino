#include <Arduino.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>
#include "Cdrv8833.h"

#define rightMotor0 25
#define rightMotor0Dir HIGH
#define rightMotor1 26
#define rightMotor1Dir HIGH

#define leftMotor0 33
#define leftMotor0Dir HIGH
#define leftMotor1 32
#define leftMotor1Dir HIGH

#define armMotor0 21
#define armMotor0Dir HIGH
#define armMotor1 19
#define armMotor1Dir HIGH

#define bucketServoPin 23
#define clawServoPin 22

#define auxLights0 16
#define auxLights1 17

int8_t rightMotorReverse = 1;
int8_t leftMotorReverse = 1;
int8_t armMotorReverse = 1;
Cdrv8833 rightMotor;
Cdrv8833 leftMotor;
Cdrv8833 armMotor;

Servo bucketServo;
Servo clawServo;

volatile int bucketServoDelay = 7;
volatile unsigned long bucketServoLastMove = 0;
volatile int bucketServoMax = 170;
volatile int bucketServoMin = 10;
volatile int bucketServoValue = bucketServoMax;

volatile int clawServoDelay = 7;
volatile unsigned long clawServoLastMove = 0;
volatile int clawServoMax = 140;
volatile int clawServoMin = 10;
volatile int clawServoValue = clawServoMax;

volatile bool auxLightsOn = true;
volatile int moveClawServoSpeed = 0;
volatile int moveBucketServoSpeed = 0;

volatile unsigned long lastWiggleTime = 0;
volatile int wiggleCount = 0;
volatile int wiggleDirection = 1;
unsigned long wiggleDelay = 100;
volatile bool shouldWiggle = false;

ControllerPtr controller;

// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
  if (controller == nullptr) {
    Serial.printf("CALLBACK: Controller is connected");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    controller = ctl;
    ctl->setColorLED(255, 0, 0);
    shouldWiggle = true;
    ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */);
  } else {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (controller == ctl) {
    Serial.printf("CALLBACK: Controller disconnected");
    controller = nullptr;
  } else {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  int LXValue = ctl->axisX();
  int LYValue = ctl->axisY();
  int8_t driveInput = -map(LYValue, -512, 511, -100, 100);
  int8_t steeringInput = map(LXValue, -512, 511, -100, 100);

  int8_t leftMotorSpeed = max(min(driveInput - steeringInput, 100), -100);
  int8_t rightMotorSpeed = max(min(driveInput + steeringInput, 100), -100);

  leftMotor.move(leftMotorSpeed);
  rightMotor.move(rightMotorSpeed);
  if (LXValue > -2 && LXValue < 2 && LYValue > -2 && LYValue < 2) {
    // Stick centered, stop movement
    rightMotor.stop();
    leftMotor.stop();
  }

  int RXValue = (ctl->axisRX());
  int RYValue = (ctl->axisRY());
  if (abs(RXValue) + abs(RYValue) > 2) {
    int8_t armSpeed = map(RYValue, -512, 511, -100, 100);
    armMotor.move(armSpeed);
  }
  if (RYValue > -30 && RYValue < 30) {
    armMotor.stop();
  }

  // Check shoulder to move claw
  if (ctl->l1() && ctl->r1() || !ctl->l1() && !ctl->r1()) {
    moveClawServoSpeed = 0;
  }
  if (ctl->l1()) {
    moveClawServoSpeed = 1;
  }
  if (ctl->r1()) {
    moveClawServoSpeed = -1;
  }

  // Check throttle to move bucket
  if (ctl->l2() && ctl->r2() || !ctl->l2() && !ctl->r2()) {
    moveBucketServoSpeed = 0;
  }
  if (ctl->l2()) {
    moveBucketServoSpeed = 1;
  }
  if (ctl->r2()) {
    moveBucketServoSpeed = -1;
  }

  if (ctl->a()) {
    shouldWiggle = true;
  }

  if (ctl->thumbR()) {
    if (!auxLightsOn) {
      digitalWrite(auxLights0, HIGH);
      digitalWrite(auxLights1, HIGH);
      auxLightsOn = true;
    } else {
      digitalWrite(auxLights0, LOW);
      digitalWrite(auxLights1, LOW);
      auxLightsOn = false;
    }
  }
}

void processControllers() {
  if (controller && controller->isConnected() && controller->hasData()) {
    if (controller->isGamepad()) {
      processGamepad(controller);
    } else {
      Serial.println("Unsupported controller");
    }
  }
}

void setup() {

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);


  Serial.println("Ready.");

  rightMotor.init(rightMotor0, rightMotor1, 5);
  rightMotor.setDecayMode(drv8833DecaySlow);
  rightMotor.swapDirection(false);
  leftMotor.init(leftMotor0, leftMotor1, 6);
  leftMotor.setDecayMode(drv8833DecaySlow);
  leftMotor.swapDirection(true);
  armMotor.init(armMotor0, armMotor1, 7);
  armMotor.setDecayMode(drv8833DecaySlow);

  pinMode(auxLights0, OUTPUT);
  pinMode(auxLights1, OUTPUT);
  digitalWrite(auxLights0, HIGH);
  digitalWrite(auxLights1, HIGH);

  bucketServo.attach(bucketServoPin);
  clawServo.attach(clawServoPin);

  bucketServo.write(bucketServoValue);
  clawServo.write(clawServoValue);
}

void wiggle() {
  unsigned long currentTime = millis();
  if (abs((int)(currentTime - lastWiggleTime)) >= wiggleDelay) {
    lastWiggleTime = currentTime;
    wiggleDirection = -wiggleDirection;
    wiggleCount++;
    rightMotor.move(wiggleDirection * 100);
    leftMotor.move(-1 * wiggleDirection * 100);
    if (wiggleCount >= 10) {
      rightMotor.brake();
      leftMotor.brake();
      wiggleCount = 0;
      shouldWiggle = false;
    }
  }
}

void loop() {
  unsigned long currentTime = millis();
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  if (shouldWiggle) {
    wiggle();
  }
  if (moveClawServoSpeed != 0) {
    if (currentTime - clawServoLastMove >= clawServoDelay) {
      clawServoLastMove = currentTime;
      int newClawServoValue = clawServoValue + moveClawServoSpeed;
      if (newClawServoValue >= clawServoMin && newClawServoValue <= clawServoMax) {
        clawServoValue = newClawServoValue;
        clawServo.write(clawServoValue);
      }
    }
  }
  if (moveBucketServoSpeed != 0) {
    if (currentTime - bucketServoLastMove > bucketServoDelay) {
      bucketServoLastMove = currentTime;
      int newBucketServoValue = bucketServoValue + moveBucketServoSpeed;
      if (newBucketServoValue >= bucketServoMin && newBucketServoValue <= bucketServoMax) {
        bucketServoValue = newBucketServoValue;
        bucketServo.write(bucketServoValue);
      }
    }
  }
}