/**
 * @file ESP32_Serial_Communication.c
 * @author Jules
 * @brief C code for ESP32 to communicate with a VEX V5 Brain via MAX485.
 * @version 0.1
 * @date 2025-09-26
 *
 * @copyright Copyright (c) 2025
 *
 * This code is intended for use with the Arduino IDE or a similar platform
 * for ESP32 development (like PlatformIO). While it is a .c file, it uses
 * the Arduino framework's functions (`setup`, `loop`, `Serial`, etc.).
 *
 */

#include <Arduino.h>
#include <HardwareSerial.h>

// --- Pin Definitions ---
// The ESP32 has multiple UART peripherals. We will use UART2.
// The default pins for UART2 on many ESP32 boards are GPIO16 (RX2) and GPIO17 (TX2).
// Check your specific ESP32 board's pinout to be sure.
#define RXD2_PIN 16
#define TXD2_PIN 17

// This pin will control the MAX485's direction (DE and /RE pins).
#define DIRECTION_PIN 4 // You can use any available GPIO pin.

// --- Hardware Setup Instructions ---
// 1. ESP32 UART Pins <--> MAX485 RO and DI pins.
//    - Connect ESP32's TXD2_PIN (GPIO 17) to MAX485 DI (Driver Input).
//    - Connect ESP32's RXD2_PIN (GPIO 16) to MAX485 RO (Receiver Output).
// 2. ESP32 Control Pin <--> MAX485 DE and /RE pins.
//    - Connect the DIRECTION_PIN (GPIO 4) to both DE (Driver Enable)
//      and /RE (Receiver Enable) pins of the MAX485.
// 3. Power and Ground:
//    - Connect the MAX485's VCC and GND to the ESP32's 5V/3.3V and GND.
//    - Ensure the ESP32 and VEX Brain share a common ground.

// We will use Serial2 for communication with the VEX Brain.
HardwareSerial SerialVEX(2);

/**
 * @brief Sets the MAX485 chip to transmit or receive mode.
 *
 * @param mode HIGH for transmit, LOW for receive.
 */
void set_direction(int mode) {
    digitalWrite(DIRECTION_PIN, mode);
}

/**
 * @brief Sends a single character to the VEX Brain.
 *
 * @param data The character to send.
 */
void sendData(char data) {
    // Set MAX485 to transmit mode.
    set_direction(HIGH);

    // A very small delay to allow the direction pin to settle before sending data.
    delayMicroseconds(100);

    // Write the data to the serial port connected to the VEX Brain.
    SerialVEX.write(data);

    // Wait for the serial buffer to be emptied. This ensures the message is
    // fully sent before we switch back to receive mode.
    SerialVEX.flush();

    // A small delay to ensure the last bit has left the transceiver.
    delay(5);

    // Set MAX485 back to receive mode.
    set_direction(LOW);
}

/**
 * @brief The main setup function, runs once after boot.
 */
void setup() {
    // Start the standard serial port for debugging (e.g., to the Arduino IDE's Serial Monitor).
    Serial.begin(115200);
    Serial.println("\nESP32-VEX Communication Initialized");

    // Configure the direction control pin as an output.
    pinMode(DIRECTION_PIN, OUTPUT);

    // Start the serial port for VEX communication.
    // The baud rate MUST match the rate set on the VEX V5 Brain (9600).
    SerialVEX.begin(9600, SERIAL_8N1, RXD2_PIN, TXD2_PIN);

    // Set the initial direction to receive.
    set_direction(LOW);

    Serial.println("Status: Ready to send data.");
}

/**
 * @brief The main loop, runs repeatedly.
 */
void loop() {
    // Let's create a character to send. We'll cycle through A, B, C...
    static char charToSend = 'A';

    // 1. Send a character to the VEX Brain.
    Serial.printf("Sending: '%c'\n", charToSend);
    sendData(charToSend);

    // 2. Wait for the VEX Brain to echo the character back.
    // We'll wait for up to 1 second for a response.
    unsigned long startTime = millis();
    bool responseReceived = false;
    Serial.print("Waiting for echo... ");

    while (millis() - startTime < 1000) {
        if (SerialVEX.available()) {
            char receivedData = SerialVEX.read();
            Serial.printf("Received: '%c'\n", receivedData);

            // Check if the echo is correct.
            if (receivedData == charToSend) {
                Serial.println("Echo is correct!");
            } else {
                Serial.println("Error: Echo does not match!");
            }

            responseReceived = true;
            break; // Exit the waiting loop
        }
    }

    if (!responseReceived) {
        Serial.println("No response from VEX Brain.");
    }

    // Prepare the next character to send.
    charToSend++;
    if (charToSend > 'Z') {
        charToSend = 'A';
    }

    // Wait for 2 seconds before starting the next cycle.
    Serial.println("--------------------");
    delay(2000);
}