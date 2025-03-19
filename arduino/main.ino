#include <DmxMaster.h>

const int spotlights[] = {125, 137}; // Starting DMX channels for each spotlight
const int numSpotlights = sizeof(spotlights) / sizeof(spotlights[0]);

void setup() {
  Serial.begin(9600); 

  // Initialize DMX communication 
  DmxMaster.usePin(3);

  // Ensure both fixtures are within range
  DmxMaster.maxChannel(150); 

  for (int i = 0; i < numSpotlights; i++) {
    int base = spotlights[i];
    
    // Initialize fixture settings for each spotlight
    DmxMaster.write(base, 0);         // Pan motor movement
    DmxMaster.write(base + 1, 255);   // Pan motor fine
    DmxMaster.write(base + 2, 0);     // Tilt motor movement
    // Tilt motor fine
    DmxMaster.write(base + 4, 10); // All dimming adjustments from dark to bright
    DmxMaster.write(base + 5, 255); // Red color adjustment from dark to bright
    DmxMaster.write(base + 6, 255); // Green color adjustment from dark to bright
    DmxMaster.write(base + 7, 255); // Blue color adjustment from dark to bright
    // DmxMaster.write(base + 8, 255); // White color adjustment from dark to bright
    // Adjust mixed color
    // Adjust strobe from slow to fast
    // Close/open sound (0-127 = close and 128-255 = open)
  }
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');  
    processSerialData(data);
  }
}

void processSerialData(String data) {
  int start = 0;
  while (start < data.length()) {
    int sepIndex = data.indexOf(';', start);
    if (sepIndex == -1) sepIndex = data.length(); // Last segment

    String segment = data.substring(start, sepIndex);
    int colonIndex = segment.indexOf(':');
    int commaIndex = segment.indexOf(',');

    if (colonIndex > 0 && commaIndex > colonIndex) {
      int id = segment.substring(0, colonIndex).toInt();
      int panValue = segment.substring(colonIndex + 1, commaIndex).toInt();
      int tiltValue = segment.substring(commaIndex + 1).toInt();

      if (id >= 0 && id < numSpotlights) {
        int base = spotlights[id];
        DmxMaster.write(base, panValue);
        DmxMaster.write(base + 2, tiltValue);
        DmxMaster.write(base + 3, 255);
      }
    }
    start = sepIndex + 1; // Move to next segment
  }
}
