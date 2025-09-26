#include <Arduino.h>
#include "HX711.h"
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;
float x = 0;
int y = 0;

HX711 scale;

void setup() {
  Serial.begin(1040);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(103);
  scale.tare();               // reset the scale to 0
}
void loop() {
  x = scale.get_units();
  Serial.print(x, 1);
  y= x;
  if (y > 420){
    Serial.print("@");
  }
  Serial.print("?");

  delay(1500);
}