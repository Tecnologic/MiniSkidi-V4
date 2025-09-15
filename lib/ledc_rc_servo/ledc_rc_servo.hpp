#ifndef LEDC_RC_SERVO_HPP
#define LEDC_RC_SERVO_HPP

#include <Arduino.h>
#include <driver/ledc.h>
#include <esp_log.h>


/**
 * @brief Class to control an RC servo using the ESP32 LEDC (LED Control) PWM functionality.
 * 
 * This class allows you to control an RC servo by specifying the pin, minimum and maximum pulse widths.
 * It uses the LEDC peripheral of the ESP32 to generate the required PWM signals.
 * 
 * Example usage:
 * ```cpp
 * LedcRCServo myServo(5, 1000, 2000); //
 * Create a servo on pin 5 with pulse widths from 1000 to 2000 microseconds
 * myServo.setup(); // Initialize the servo
 * myServo.write(500); // Set servo to 75% of its range
 * int32_t position = myServo.read(); // Read the current position
 * Serial.println(position); // Print the position
 * ```  
 */
class LedcRCServo
{
public:
    LedcRCServo(const uint8_t pin, uint16_t minPulseWidth, uint16_t maxPulseWidth)
        : servoPin(pin), minPulseWidth(minPulseWidth), maxPulseWidth(maxPulseWidth), ledcChannel(nextChannel)
    {
        nextChannel++;

        if (nextChannel >= maxChannels)
        {
            ESP_LOGE("LedcRCServo", "Exceeded maximum number of LEDC channels (%d).", maxChannels);
            nextChannel = 0; // Wrap around if exceeding max channels
        }
    }

    /**
     * @brief Initialize the servo by setting up the LEDC channel and attaching the pin.
     */
    void setup ( void ) {
        ESP_LOGI("LedcRCServo", "Setting up servo on pin %d with channel %d", servoPin, ledcChannel);

        ledcSetup(ledcChannel, ledcFrequency, ledcResolution);

        ledcAttachPin(servoPin, ledcChannel);

        write(0); // Initialize to midpoint
    }

    /**
     * @brief Write a value between -512 and 512 to the servo.
     * -1000 corresponds to minPulseWidth, 0 to the midpoint, and 1000 to maxPulseWidth.
     * @param value Value between -512 and 512
     */
    void write(const int32_t value) {
        uint32_t microseconds = map(value, -512, 512, minPulseWidth, maxPulseWidth);
        ledcWrite(ledcChannel, microsecoundsToPulseWidth(microseconds));
    }

    /**
     * @brief Read the current position of the servo as a pulse width in microseconds.
     * @return Current pulse width value in microseconds
     */
    int32_t read( void ) {
        return pulseWidthtoMicroseconds(ledcRead(ledcChannel));
    }

private:

    /**
     * @brief Convert a pulse width value to microseconds.
     * @param value Pulse width value
     * @return Pulse width in microseconds
     */
    static constexpr int32_t pulseWidthtoMicroseconds(const int32_t value) {
        return (value * ((1 << ledcResolution) - 1)) / (1000000 / ledcFrequency);
    }

    /**
     * @brief Write a pulse width in microseconds to the servo.
     * @param microseconds Pulse width in microseconds
     */
    int32_t microsecoundsToPulseWidth(const int32_t microseconds) {
        return (microseconds * (1000000 / ledcFrequency)) / ((1 << ledcResolution) - 1);
    }


    static uint8_t nextChannel; // Static variable to keep track of the next available LEDC channel
    const uint8_t servoPin;
    const uint16_t minPulseWidth;
    const uint16_t maxPulseWidth;
    const uint8_t ledcChannel;

    static constexpr uint32_t ledcFrequency = 50; // 50 Hz for servo control
    static constexpr uint8_t ledcResolution = 14; // 14-bit resolution
    static constexpr uint8_t maxChannels = 6; // ESP32-C3 has 6 LEDC channels (0-5)
};

#endif // LEDC_RC_SERVO_HPP