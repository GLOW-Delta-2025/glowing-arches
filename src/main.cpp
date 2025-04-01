#include <DmxSimple.h>
#include "Arduino.h"

const int spotlights[] = { 125, 137 };  // Starting DMX channels for each spotlight
const int numSpotlights = sizeof(spotlights) / sizeof(spotlights[0]);

int getValue(String &values, int index) {
  int commaIndex = values.indexOf(',');
  String value;

  if (commaIndex != -1) {
    value = values.substring(0, commaIndex); // Get the value before the comma
    values = values.substring(commaIndex + 1); // Remove the value from the string
  } else {
    value = values; // Get the last value if no comma is found
  }
  
  return value.toInt(); // Convert string to integer
}

void setup() {
  Serial.begin(9600);

  // Initialize DMX communication
  DmxSimple.usePin(2);

  // Ensure both fixtures are within range
  DmxSimple.maxChannel(150);

  for (int i = 0; i < numSpotlights; i++) {
    int base = spotlights[i];

    // Initialize fixture settings for each spotlight
    DmxSimple.write(base, 0);       // Pan motor movement
    DmxSimple.write(base + 1, 255); // Pan motor fine
    DmxSimple.write(base + 2, 0);   // Tilt motor movement
    // Tilt motor fine
    DmxSimple.write(base + 4, 150);  // All dimming adjustments from dark to bright
    DmxSimple.write(base + 5, 0);   // Full Red
    DmxSimple.write(base + 6, 0);   // No Green
    DmxSimple.write(base + 7, 0);   // No Blue
    DmxSimple.write(base + 8, 0);   // White color adjustment from dark to bright
    DmxSimple.write(base + 9, 0);   // Adjust mixed color
    // Adjust mixed color
    // Adjust strobe from slow to fast
    // Close/open sound (0-127 = close and 128-255 = open)
  }
}

void processSerialData(String data) {
  int start = 0;
  while (start < data.length()) {
    int sepIndex = data.indexOf(';', start);
    if (sepIndex == -1) sepIndex = data.length();  // Last segment

    String segment = data.substring(start, sepIndex);

    int colonIndex = segment.indexOf(':');
    if (colonIndex > 0) {
      // Extract ID and data values
      int id = segment.substring(0, colonIndex).toInt();
      String values = segment.substring(colonIndex + 1);
      
      // Split values by commas
      int panValue = getValue(values, 0);
      int tiltValue = getValue(values, 1);
      int redValue = getValue(values, 2);
      int greenValue = getValue(values, 3);
      int blueValue = getValue(values, 4);
      int whiteValue = getValue(values, 5);
      int mixedColorValue = getValue(values, 6);
      int dimmingValue = getValue(values, 7);

      // Print the values for debugging
      Serial.println("ID: " + String(id));
      Serial.println("Pan: " + String(panValue));
      Serial.println("Tilt: " + String(tiltValue));
      Serial.println("Red: " + String(redValue));
      Serial.println("Green: " + String(greenValue));
      Serial.println("Blue: " + String(blueValue));
      Serial.println("White: " + String(whiteValue));
      Serial.println("Mixed Color: " + String(mixedColorValue));
      Serial.println("Dimming: " + String(dimmingValue));

      // Apply the DMX values
      if (id >= 0 && id < numSpotlights) {
        int base = spotlights[id];
        DmxSimple.write(base, panValue);
        DmxSimple.write(base + 2, tiltValue);
        DmxSimple.write(base + 4, dimmingValue);
        DmxSimple.write(base + 5, redValue);
        DmxSimple.write(base + 6, greenValue);
        DmxSimple.write(base + 7, blueValue);
        DmxSimple.write(base + 8, whiteValue);
        DmxSimple.write(base + 9, mixedColorValue);
      }
    }
    start = sepIndex + 1;  // Move to the next segment
  }
}

void loop() {
    if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n');
      processSerialData(data);
    }
  }