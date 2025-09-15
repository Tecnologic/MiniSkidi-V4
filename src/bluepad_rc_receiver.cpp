#include <Arduino.h>
#include <Bluepad32.h>
#include <esp_log.h>
#include "ledc_rc_servo.hpp"

const int ledPin = 8; // Der GPIO-Pin, an den die LED angeschlossen ist


LedcRCServo servoLY(5, 1000, 2000); // servo on pin 4 with 1ms to 2ms pulse width on Y axis of left stick
LedcRCServo servoLX(6, 1000, 2000); // servo on pin 5 with 1ms to 2ms pulse width on X axis of left stick
LedcRCServo servoRY(7, 1000, 2000); // servo on pin 6 with 1ms to 2ms pulse width on Y axis of right stick
LedcRCServo servoRX(8, 1000, 2000); // servo on pin 7 with 1ms to 2ms pulse width on X axis of right stick

ControllerPtr controller = nullptr;         // Pointer to the connected controller

/**
 * @brief Callback for when a controller is connected
 * This function checks if we have an empty slot for a new controller.
 * If so, it assigns the controller to the slot, sets the LED color to red,
 * and triggers a rumble effect.
 * @param ctl Pointer to the connected controller
 */
void onConnectedController(ControllerPtr ctl)
{
  if (controller == nullptr)
  {
    Serial.printf("CALLBACK: Controller is connected");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    controller = ctl;
    ctl->setColorLED(255, 0, 0);
    ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */);
  }
  else
  {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

/**
 * @brief Callback for when a controller is disconnected
 * This function checks if the disconnected controller is the one we are using.
 * If so, it resets the controller pointer and sets all servos to neutral position.
 * @param ctl Pointer to the disconnected controller
 */
void onDisconnectedController(ControllerPtr ctl)
{
  if (controller == ctl && controller != nullptr)
  {
    Serial.printf("CALLBACK: Controller disconnected");
    controller = nullptr;
    servoLX.write(0);
    servoLY.write(0);
    servoRX.write(0);
    servoRY.write(0);
  }
  else
  {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

/**
 * @brief Process gamepad inputs
 * This function reads the axis values from the gamepad and writes them to the servos.
 * @param ctl Pointer to the connected controller
 */
void processGamepad(ControllerPtr ctl)
{
  servoLX.write(ctl->axisX());
  servoLY.write(ctl->axisY());
  servoRX.write(ctl->axisRX());
  servoRY.write(ctl->axisRY());
}

/**
 * @brief Process connected controllers
 * This function checks if a controller is connected and has data.
 * If so, it processes the gamepad inputs.
 */
void processControllers()
{
  if (controller && controller->isConnected() && controller->hasData())
  {
    if (controller->isGamepad())
    {
      processGamepad(controller);
    }
    else
    {
      Serial.println("Unsupported controller");
    }
  }
}

/**
 * @brief Setup function
 * This function initializes the serial communication, sets up the Bluepad32 library,
 * and initializes the servos.
 */
void setup()
{
// Note: Do not use Serial because esp log uses it too
//  Serial.begin(115200);
  ESP_LOGI("Bluepad32", "Starting Bluepad32 RC Receiver");
  ESP_LOGI("Bluepad32", "Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  ESP_LOGI("Bluepad32", "BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  servoLX.setup();
  servoLY.setup();
  servoRX.setup();
  servoRY.setup();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); // LED an

  ESP_LOGI("rc_receiver", "Ready.");
}

/**
 * @brief Main loop
 * This loop continuously checks for updates from the Bluepad32 library and processes controller inputs.
 * It also prints the current servo positions every second.
 * 
 */
void loop()
{
  unsigned long currentTime = millis();
  bool dataUpdated = BP32.update();
  if (dataUpdated)
  {
    processControllers();
  }

  if (controller == nullptr)
  {
    digitalWrite(ledPin, (currentTime / 250) % 2 == 0 ? HIGH : LOW); // LED blinks fast when no controller is connected

    if (currentTime % 1000 == 0)
    {
          ESP_LOGW("Bluepad32", "No controller connected");
          ESP_LOGI("rc_receiver", "LEDC positions: LX=%d, LY=%d, RX=%d, RY=%d\n",
                    servoLX.read(),
                    servoLY.read(),
                    servoRX.read(),
                    servoRY.read());
    }

    return;
  }
  else
  {
    digitalWrite(ledPin, (currentTime / 1000) % 2 == 0 ? HIGH : LOW); // LED on when controller is connected

    if (currentTime % 1000 == 0)
    {
      digitalWrite(ledPin, !digitalRead(ledPin)); // LED toggles every second
      ESP_LOGI("rc_receiver", "Servo positions: LX=%d, LY=%d, RX=%d, RY=%d\n",
                    controller ? controller->axisX() : 0,
                    controller ? controller->axisY() : 0,
                    controller ? controller->axisRX() : 0,
                    controller ? controller->axisRY() : 0);

      ESP_LOGI("rc_receiver", "LEDC positions: LX=%dus, LY=%dus, RX=%dus, RY=%dus\n",
                     servoLX.read(),
                     servoLY.read(),
                     servoRX.read(),
                     servoRY.read());
    }
  }
}