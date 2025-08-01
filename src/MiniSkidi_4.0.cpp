#include <Arduino.h>
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

#define batteryPin 34 // ADC pin for battery voltage measurement


constexpr int32_t minBatteryVoltage = 6600; // Minimum battery voltage in millivolts (6,6V = 2S LiPo fully discharged)
constexpr int32_t warnBatteryVoltage = 7000; // Warning battery voltage in millivolts (7,0V = 2S LiPo low warning threshold)

int8_t rightMotorReverse = 1;
int8_t leftMotorReverse = 1;
int8_t armMotorReverse = 1;
Cdrv8833 rightMotor;
Cdrv8833 leftMotor;
Cdrv8833 armMotor;


constexpr int bucketServoMax = 2000;  // Maximum pulse width for bucket servo in microseconds
constexpr int bucketServoMin = 1000;  // Minimum pulse width for bucket servo in microseconds
int bucketServoSpeed = 0;  // Speed for bucket servo, can be adjusted based on input
int bucketServoValue = bucketServoMin;  // Initial value for bucket servo

constexpr int clawServoMax = 2000;    // Maximum pulse width for claw servo in microseconds
constexpr int clawServoMin = 1000;   // Minimum pulse width for claw servo in microseconds
int clawServoSpeed = 0;  // Speed for claw servo, can be adjusted based on input
int clawServoValue = clawServoMax;    // Initial value for claw servo

volatile bool auxLightsOn = true;

volatile unsigned long lastWiggleTime = 0;
volatile int wiggleCount = 0;
volatile int wiggleDirection = 1;
unsigned long wiggleDelay = 100;
volatile bool shouldWiggle = false;
volatile bool yPressed = false;

ControllerPtr controller;

void bucketServoWrite(int value) {
  // Convert the value to a range suitable for the servo
  int servoValue = map(value, 0, 20000, 0, 65535); // 20000 is the max pulse width in microseconds for 50Hz PWM
  // Ensure the value is within the range of 0 to 65535 for 16-bit PWM
  servoValue = constrain(servoValue, 0, 65535);
  // Write the value to the LEDC channel for the bucket servo
  ledcWrite(0, servoValue);
}

void clawServoWrite(int value) {
  // Convert the value to a range suitable for the servo
  int servoValue = map(value, 0, 20000, 0, 65535); // 20000 is the max pulse width in microseconds for 50Hz PWM
  // Ensure the value is within the range of 0 to 65535 for 16-bit PWM
  servoValue = constrain(servoValue, 0, 65535);
  // Write the value to the LEDC channel for the claw servo
  ledcWrite(1, servoValue);
}


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
    rightMotor.stop();
    leftMotor.stop();
    armMotor.stop();
    bucketServoSpeed = 0;
    clawServoSpeed = 0;
    bucketServoWrite(bucketServoValue);
    clawServoWrite(clawServoValue);
    digitalWrite(auxLights0, LOW);
    digitalWrite(auxLights1, LOW);
  } else {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }

}

void processGamepad(ControllerPtr ctl) {
  int LXValue = ctl->axisX();
  int LYValue = ctl->axisY();

  if (abs(LXValue) > 20 || abs(LYValue) > 20)
  {
    int8_t driveInput = -map(LYValue, -512, 511, -100, 100);
    int8_t steeringInput = map(LXValue, -512, 511, -100, 100);

    int8_t leftMotorSpeed = max(min(driveInput - steeringInput, 100), -100);
    int8_t rightMotorSpeed = max(min(driveInput + steeringInput, 100), -100);

    leftMotor.move(leftMotorSpeed);
    rightMotor.move(rightMotorSpeed);
  }
  else
  {
    // Stick centered, stop movement
    rightMotor.stop();
    leftMotor.stop();
  }

  int RYValue = (ctl->axisRY());
  if (abs(RYValue) > 20) {
    int8_t armSpeed = map(RYValue, -512, 511, -100, 100);
    armMotor.move(armSpeed);
  }
  else{
    armMotor.stop();
  }

  int RXValue = (ctl->axisRX());
  if (abs(RXValue) > 20) {
    bucketServoSpeed = map(RXValue, -512, 511, -100, 100);
  }
  else {
    bucketServoSpeed = 0;
  }

  int ThrottleValue = ctl->throttle();
  int BrakeValue = ctl->brake();
  if (abs(ThrottleValue) > 20 || abs(BrakeValue) > 20) {
    clawServoSpeed = map(ThrottleValue, 0, 1024, 0, 100) - map(BrakeValue, 0, 1024, 0, 100);
  }
  else {
    clawServoSpeed = 0;
  }

  if (ctl->a()) {
    shouldWiggle = true;
  }

  if (ctl->y() && !yPressed)
  {
    yPressed = true;

    if (!auxLightsOn)
    {
      digitalWrite(auxLights0, HIGH);
      digitalWrite(auxLights1, HIGH);
      auxLightsOn = true;
    }
    else
    {
      digitalWrite(auxLights0, LOW);
      digitalWrite(auxLights1, LOW);
      auxLightsOn = false;
    }
  } else if (!ctl->y() && yPressed) {
    yPressed = false;
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


int32_t readBatteryVoltage() {
  constexpr int32_t adcMaxValue = 4095; // ESP32 ADC max value for 12-bit resolution
  constexpr int32_t adcMaxVoltage = 3300 * (13000 + 4700) / 4700; // 3300 mV reference voltage, voltage divider with 12k and 4.7k resistors
  // Read the battery voltage from the ADC pin
  int rawValue = analogRead(batteryPin);

  return (rawValue * adcMaxVoltage / adcMaxValue); // Convert ADC value to voltage in millivolts
}


void setup() {

  
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  Serial.println("MiniSkidi 4.0 starting...");
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

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

  pinMode(bucketServoPin, OUTPUT);
  pinMode(clawServoPin, OUTPUT);
  ledcSetup(0, 50, 16); // Set up PWM for servos
  ledcSetup(1, 50, 16); // Set up PWM for servos

  ledcAttachPin(bucketServoPin, 0);
  ledcAttachPin(clawServoPin, 1);

  bucketServoWrite(bucketServoValue);
  clawServoWrite(clawServoValue);

  pinMode(batteryPin, ANALOG); // Set up battery pin as analog input

  Serial.println("Ready.");
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

  int32_t batteryVolts = readBatteryVoltage();

  if (currentTime % 1000 == 0) {
    Serial.printf("Battery voltage: %d mV\n", batteryVolts);
  }

  if (batteryVolts < warnBatteryVoltage)
  {
    // Blink aux lights to indicate low battery
    if (currentTime % 500 < 250)
    {
      digitalWrite(auxLights0, HIGH);
      digitalWrite(auxLights1, HIGH);
    }
    else
    {
      digitalWrite(auxLights0, LOW);
      digitalWrite(auxLights1, LOW);
    }

    if (batteryVolts < minBatteryVoltage)
    {
      if (currentTime % 500 == 0)
      {
        Serial.println("Battery voltage is low, stopping motors and servos.");
      }
      rightMotor.stop();
      leftMotor.stop();
      armMotor.stop();
      bucketServoSpeed = 0;
      clawServoSpeed = 0;
      bucketServoWrite(bucketServoValue);
      clawServoWrite(clawServoValue);
    }
    else if (currentTime % 500 == 0)
    {
      Serial.printf("Warning: Battery voltage is low (%d mV), consider recharging.\n", batteryVolts);
    }
  }

  if (currentTime % 10 == 0)
  {
    constexpr int tmpMultiplier = 100; // For converting servo values to 1/100 microseconds

    static int tmpBucketServoValue = bucketServoMax;
    static int tmpClawServoValue = clawServoMax;
    // Update bucket and claw servo values based on speed
    tmpBucketServoValue += bucketServoSpeed;
    tmpBucketServoValue = constrain(tmpBucketServoValue, bucketServoMin * tmpMultiplier, bucketServoMax * tmpMultiplier);

    tmpClawServoValue += clawServoSpeed;
    tmpClawServoValue = constrain(tmpClawServoValue, clawServoMin * tmpMultiplier, clawServoMax * tmpMultiplier);
    // Write the updated values to the servos
    bucketServoValue = tmpBucketServoValue / tmpMultiplier;
    clawServoValue = tmpClawServoValue / tmpMultiplier;
    bucketServoValue = constrain(bucketServoValue, bucketServoMin, bucketServoMax);
    clawServoValue = constrain(clawServoValue, clawServoMin, clawServoMax);
    bucketServoWrite(bucketServoValue);
    clawServoWrite(clawServoValue);
  }
}