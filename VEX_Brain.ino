#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "vex.h"

using namespace vex;

brain Brain;
motor Arduino = motor(PORT1, ratio18_1, false);

#define waitUntil(condition) \
do { \
wait(5, msec); \
 } while (!(condition))

#define repeat(iterations) \
for (int iterator = 0; iterator < iterations; iterator++)

#define MAX_SCAN_POINTS 181
#define SCREEN_CENTER_X 240
#define SCREEN_CENTER_Y 180
#define MAX_DISPLAY_DISTANCE 150
#define SCALE_FACTOR (double)(SCREEN_CENTER_Y / MAX_DISPLAY_DISTANCE)

#define END_FORWARD_SWEEP 0xFE
#define END_BACKWARD_SWEEP 0xFF

struct ScanPoint {
  float x;
  float y;
  bool valid;
};

ScanPoint scanData[MAX_SCAN_POINTS];
bool newScanComplete = false;
int packetCount = 0;

#define BAUDRATE 115200

void drawLidarScan() {
  Brain.Screen.setPenColor(black);
  Brain.Screen.setFillColor(black);
  Brain.Screen.drawRectangle(0, 0, 480, 240);
  
  Brain.Screen.setPenColor(vex::color(128, 128, 128));
  Brain.Screen.setPenWidth(1);
  Brain.Screen.drawLine(SCREEN_CENTER_X, 0, SCREEN_CENTER_X, 240);
  Brain.Screen.drawLine(SCREEN_CENTER_X, SCREEN_CENTER_Y, 480, SCREEN_CENTER_Y);
  Brain.Screen.drawLine(SCREEN_CENTER_X, SCREEN_CENTER_Y, 
                        SCREEN_CENTER_X + (int)(SCREEN_CENTER_Y * 0.7), 
                        SCREEN_CENTER_Y - (int)(SCREEN_CENTER_Y * 0.7));
  Brain.Screen.drawLine(SCREEN_CENTER_X, SCREEN_CENTER_Y, 
                        SCREEN_CENTER_X + (int)(SCREEN_CENTER_Y * 0.7), 
                        SCREEN_CENTER_Y + (int)(SCREEN_CENTER_Y * 0.7));
  
  Brain.Screen.setPenColor(white);
  Brain.Screen.setCursor(1, 25);
  Brain.Screen.print("0°");
  Brain.Screen.setCursor(18, 47);
  Brain.Screen.print("90°");
  Brain.Screen.setCursor(23, 25);
  Brain.Screen.print("180°");
  Brain.Screen.setCursor(8, 40);
  Brain.Screen.print("45°");
  Brain.Screen.setCursor(20, 40);
  Brain.Screen.print("135°");
  
  Brain.Screen.setPenColor(vex::color(64, 64, 64));
  for(int d = 50; d <= MAX_DISPLAY_DISTANCE; d += 50) {
    int radius = d * SCALE_FACTOR;
    for(int angle = 0; angle <= 180; angle += 2) {
      int x = SCREEN_CENTER_X + (int)(radius * cos(angle * M_PI / 180.0));
      int y = SCREEN_CENTER_Y - (int)(radius * sin(angle * M_PI / 180.0));
      Brain.Screen.drawPixel(x, y);
    }
    
    Brain.Screen.setPenColor(white);
    Brain.Screen.setCursor((SCREEN_CENTER_Y-radius)/10, (SCREEN_CENTER_X + 20)/10);
    Brain.Screen.print("%d", d);
  }
  
  Brain.Screen.setPenColor(vex::color(0, 255, 0));
  Brain.Screen.setPenWidth(3);
  
  int validPointCount = 0;
  
  for(int i = 0; i < MAX_SCAN_POINTS; i++) {
    if(scanData[i].valid) {
      validPointCount++;
      int screenX = SCREEN_CENTER_X + (int)(scanData[i].x * SCALE_FACTOR);
      int screenY = SCREEN_CENTER_Y - (int)(scanData[i].y * SCALE_FACTOR);
      
      if(screenX >= 0 && screenX < 480 && screenY >= 0 && screenY < 240) {
        Brain.Screen.drawCircle(screenX, screenY, 2);
      }
    }
  }
  
  Brain.Screen.setPenColor(white);
  Brain.Screen.setCursor(22, 1);
  Brain.Screen.print("LiDAR Scan (180°) - Scale: 1px = %.1fcm", 1.0/SCALE_FACTOR);
  
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Valid points: %d", validPointCount);
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Packets received: %d", packetCount);
}

void readArduinoData() {
  static uint8_t buffer[10];
  static int bufferIndex = 0;
  static bool foundHeader = false;
  
  while (vexGenericSerialReceiveAvail(Arduino.index()) > 0) {
    uint8_t byte;
    vexGenericSerialReceive(Arduino.index(), &byte, 1);
    
    if (!foundHeader) {
      if (byte == 0xAA) {
        if (bufferIndex == 0) {
          buffer[bufferIndex++] = byte;
        } else {
          buffer[0] = byte;
          bufferIndex = 1;
        }
      } else if (byte == 0x55 && bufferIndex == 1 && buffer[0] == 0xAA) {
        buffer[bufferIndex++] = byte;
        foundHeader = true;
      } else {
        bufferIndex = 0;
      }
    } else {
      buffer[bufferIndex++] = byte;
      
      if (bufferIndex == 10) {
        foundHeader = false;
        bufferIndex = 0;
        packetCount++;
        
        uint8_t checksum = 0;
        for (int i = 0; i < 9; i++) {
          checksum += buffer[i];
        }
        
        if (checksum == buffer[9]) {
          int16_t x_mm = (buffer[3] << 8) | buffer[2];
          int16_t y_mm = (buffer[5] << 8) | buffer[4];
          uint16_t distance_mm = (buffer[7] << 8) | buffer[6];
          uint8_t angle = buffer[8];
          
          float x_cm = x_mm ;
          float y_cm = y_mm ;
          float distance_cm = distance_mm ;
          
          if (angle == END_FORWARD_SWEEP || angle == END_BACKWARD_SWEEP) {
            newScanComplete = true;
            
            Brain.Screen.setCursor(4, 1);
            Brain.Screen.print("Scan complete! Marker: 0x%02X", angle);
          } 
          else if (x_mm == 0xFFFF && y_mm == 0xFFFF && distance_mm == 0xFFFF) {
            newScanComplete = true;
            
            Brain.Screen.setCursor(4, 1);
            Brain.Screen.print("Scan complete! Special values detected");
          }
          else if (angle <= 180 && distance_mm <= 5000) {
            scanData[angle].x = x_cm;
            scanData[angle].y = y_cm;
            scanData[angle].valid = true;
            
            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("Angle: %d, Dist: %.1f cm, X: %.1f, Y: %.1f   ", 
                              angle, distance_cm, x_cm, y_cm);
          }
        } else {
          Brain.Screen.setCursor(4, 1);
          Brain.Screen.print("Checksum Error!");
        }
      }
    }
  }
}

void initScanData() {
  for(int i = 0; i < MAX_SCAN_POINTS; i++) {
    scanData[i].valid = false;
  }
}

void vexcodeInit() {
}

void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  srand(seed);
}

void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

int main() {
  vexcodeInit();
  
  vexGenericSerialEnable(Arduino.index(), 1);
  vexGenericSerialBaudrate(Arduino.index(), BAUDRATE);
  
  initScanData();
  
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("LiDAR Scanner Starting...");
  
  drawLidarScan();
  
  while (true) {
    readArduinoData();
    
    if (newScanComplete) {
      drawLidarScan();
      newScanComplete = false;
    }
    
    static int redrawCounter = 0;
    if (redrawCounter++ > 200) {
      drawLidarScan();
      redrawCounter = 0;
    }
    
    int avail = vexGenericSerialReceiveAvail(Arduino.index());
    if (avail > 100) {
      vexGenericSerialFlush(Arduino.index());
      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("Buffer flushed! (%d bytes)", avail);
    }
    
    wait(10, msec);
  }
  
  return 0;
}
