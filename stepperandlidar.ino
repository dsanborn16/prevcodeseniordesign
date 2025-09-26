#include <Arduino.h>
#include <HardwareSerial.h>
#include <Stepper.h>
#include <math.h>

// Stepper motor config
#define STEPS_PER_REV 2048
#define STEPPER_PIN_1 20
#define STEPPER_PIN_2 21
#define STEPPER_PIN_3 47
#define STEPPER_PIN_4 48

// Serial port config
#define BAUDRATE 115200

// LiDAR serial port 
#define RXD2 8  
#define TXD2 3 

// VEX serial output
#define VEX_RX 16  // VEX RX 
#define VEX_TX 17  // VEX TX 

// Global variables
Stepper stepper(STEPS_PER_REV, STEPPER_PIN_1, STEPPER_PIN_3, STEPPER_PIN_2, STEPPER_PIN_4);
HardwareSerial lidarSerial(2); 
HardwareSerial vexSerial(1);    

volatile uint16_t lidarDistance = 0;  // LiDAR distance in cm
volatile uint16_t lidarStrength = 0;  // LiDAR signal strength
volatile float lidarTemperature = 0;  // LiDAR temperature
volatile bool newData = false;        // Flag for new data available
volatile int packetsSent = 0;         // Count packets sent for debugging
volatile int currentAngle = 0;        // Current angle of the stepper
volatile bool sweepDirection = true;  // true = forward (0->180), false = backward (180->0)

// Simplified packet structure:
// [0] = 0xAA (header1)
// [1] = 0x55 (header2)
// [2-3] = x coordinate (int16_t, in mm)
// [4-5] = y coordinate (int16_t, in mm)
// [6-7] = distance (uint16_t, in mm)
// [8] = angle (uint8_t, 0-180)
// [9] = checksum

// Task to read data from LiDAR
void readLiDAR(void *pvParameters) {
    uint8_t buf[9];
    
    while (1) {
        if (lidarSerial.available() >= 9) {
            if (lidarSerial.read() == 0x59) {
                if (lidarSerial.read() == 0x59) {
                    buf[0] = 0x59;
                    buf[1] = 0x59;
                    
                    for (int i = 2; i < 9; i++) {
                        buf[i] = lidarSerial.read();
                    }
                    
                    
                    uint16_t distance = (buf[3] << 8) | buf[2];
                    uint16_t strength = (buf[5] << 8) | buf[4];
                    int16_t rawTemp = (buf[7] << 8) | buf[6];
                    float temperature = rawTemp / 8.0 - 256.0;
                    
                    // Validate with checksum
                    uint8_t checksum = 0;
                    for (int i = 0; i < 8; i++) {
                        checksum += buf[i];
                    }
                    
                    if (checksum == buf[8] && distance > 0 && distance < 12000) {
                        lidarDistance = distance;
                        lidarStrength = strength;
                        lidarTemperature = temperature;
                        newData = true;
                    }
                }
            }
        }
        vTaskDelay(1);
    }
}

void sendDataToVEX(float x_cm, float y_cm, uint16_t distance_mm, uint8_t angle) {
    int16_t x_mm = (int16_t)(x_cm * 10);
    int16_t y_mm = (int16_t)(y_cm * 10);
    //new buffer
    uint8_t buffer[10] = {
        0xAA, 0x55,  
        (uint8_t)(x_mm & 0xFF),        // X coordinate LSB
        (uint8_t)((x_mm >> 8) & 0xFF), // X coordinate MSB
        (uint8_t)(y_mm & 0xFF),        // Y coordinate LSB
        (uint8_t)((y_mm >> 8) & 0xFF), // Y coordinate MSB
        (uint8_t)(distance_mm & 0xFF),        // Distance LSB
        (uint8_t)((distance_mm >> 8) & 0xFF), // Distance MSB
        angle,                         // Angle (0-180)
        0x00                           // Checksum (will be calculated)
    };
    
    
    uint8_t checksum = 0;
    for (int i = 0; i < 9; i++) {
        checksum += buffer[i];
    }
    buffer[9] = checksum;
    
  
    vexSerial.write(buffer, 10);
    packetsSent++;
    
    
    if (packetsSent % 50 == 0) {
        Serial.printf("Sent %d packets. Last values: Angle=%d, Dist=%d, X=%.1f, Y=%.1f\n", 
                     packetsSent, angle, distance_mm, x_cm, y_cm);
    }
}


void sendEndOfSweepMarker(bool isForwardSweep) {
    uint8_t buffer[10] = {
        0xAA, 0x55,  // Header bytes
        0xFF, 0xFF,  // Special X value
        0xFF, 0xFF,  // Special Y value
        0xFF, 0xFF,  // Special distance value
        isForwardSweep ? 0xFE : 0xFF,  // Marker: 0xFE = end of forward, 0xFF = end of backward
        0x00         // Checksum (will be calculated)
    };
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 0; i < 9; i++) {
        checksum += buffer[i];
    }
    buffer[9] = checksum;
    
  
    vexSerial.write(buffer, 10);
    
    Serial.println(isForwardSweep ? "Sent end of forward sweep marker" : "Sent end of backward sweep marker");
}


void sweepStepper(void *pvParameters) {
    int stepsPerDegree = STEPS_PER_REV / 360;
    
    while (1) {
        
        for (int angle = 0; angle <= 180 && sweepDirection; angle++) {
            currentAngle = angle;
            stepper.step(stepsPerDegree);
            
            // new LiDAR data
            unsigned long startWait = millis();
            while (!newData && (millis() - startWait < 100)) {
                vTaskDelay(1);
            }
            
            if (newData) {
              
                float r = lidarDistance / 10.0;  // Convert mm to cm
                float angleRad = angle * DEG_TO_RAD;
                float x = r * cos(angleRad);
                float y = r * sin(angleRad);
                
                // Debug output
                Serial.printf("Angle: %d°, Distance: %.1f cm, X: %.1f cm, Y: %.1f cm\n",
                             angle, r, x, y);
                
                // Send data to VEX
                sendDataToVEX(x, y, lidarDistance, angle);
                
                newData = false;
            } else {
                Serial.println("Timeout waiting for LiDAR data");
            }
            
            delay(10); // Small delay between steps
        }
        
        // Signal end of forward sweep
        if (sweepDirection) {
            Serial.println("End of forward sweep (0->180)");
            sendEndOfSweepMarker(true);
            sweepDirection = false;
        }
        
        // Backward sweep (180 to 0 degrees)
        for (int angle = 180; angle >= 0 && !sweepDirection; angle--) {
            currentAngle = angle;
            stepper.step(-stepsPerDegree);
            
            // Wait for new LiDAR data
            unsigned long startWait = millis();
            while (!newData && (millis() - startWait < 100)) {
                vTaskDelay(1);
            }
            
            if (newData) {
                // Calculate X,Y coordinates
                float r = lidarDistance;  // Convert mm to cm
                float angleRad = angle * DEG_TO_RAD;
                float x = r * cos(angleRad);
                float y = r * sin(angleRad);
                
                // Debug output
                Serial.printf("Angle: %d°, Distance: %.1f cm, X: %.1f cm, Y: %.1f cm\n",
                             angle, r, x, y);
                
                sendDataToVEX(x, y, lidarDistance, angle);
                
                newData = false;
            } else {
                Serial.println("Timeout waiting for LiDAR data");
            }
            
            delay(10); 
        }
        
        // Signal end of backward sweep
        if (!sweepDirection) {
            Serial.println("End of backward sweep (180->0)");
            sendEndOfSweepMarker(false);
            sweepDirection = true;
        }
    }
}

// Test communication with VEX using simple pattern
void testVexCommunication() {
    // Send test points in a square pattern
    
    // Center point
    sendDataToVEX(0, 0, 100, 90);
    delay(100);
    
    // Four corners of a square
    sendDataToVEX(50, 50, 500, 45);
    delay(100);
    sendDataToVEX(50, -50, 500, 135);
    delay(100);
    sendDataToVEX(-50, 50, 500, 45);
    delay(100);
    sendDataToVEX(-50, -50, 500, 135);
    delay(100);
    
    // End of sweep marker
    sendEndOfSweepMarker(true);
    
    Serial.println("Sent test pattern to VEX");
}

void setup() {
    Serial.begin(115200);
    
    // Initialize serial ports
    lidarSerial.begin(BAUDRATE, SERIAL_8N1, RXD2, TXD2);  // LiDAR serial
    vexSerial.begin(BAUDRATE, SERIAL_8N1, VEX_TX, VEX_RX);  // VEX serial
    
    // Configure stepper motor
    stepper.setSpeed(5);  // 5 RPM
    
    // Wait for serial ports to initialize
    delay(1000);
    
    Serial.println("ESP32 LiDAR Scanner starting...");
    
    // Test VEX communication
    Serial.println("Testing VEX communication...");
    testVexCommunication();
    
    // Create tasks
    xTaskCreatePinnedToCore(readLiDAR, "LiDAR Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sweepStepper, "Stepper Task", 4096, NULL, 1, NULL, 1);
    
    Serial.println("Tasks started. System running.");
}

void loop() {
    // Print stats every second
    static unsigned long lastStats = 0;
    if (millis() - lastStats > 1000) {
        Serial.printf("Stats: Packets sent: %d, Last distance: %d mm, Angle: %d°\n", 
                     packetsSent, lidarDistance, currentAngle);
        lastStats = millis();
    }
    
    delay(10);
}