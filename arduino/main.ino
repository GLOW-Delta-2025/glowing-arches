#include <DmxMaster.h>

/*
DMX manual
https://www.steinigke.de/download/51785964-Anleitung-96748-1.10-eurolite-led-tmh-9-moving-head-wash-de_en.pdf
*/

const int spotlights[] = {125, 137}; // Starting DMX channels for each spotlight
const int numSpotlights = sizeof(spotlights) / sizeof(spotlights[0]); // Calculate N of spotlights

void setup() {
  // Initialize DMX communication 
  DmxMaster.usePin(3);

  // Ensure both fixtures are within range
  DmxMaster.maxChannel(150); 

  for (int i = 0; i < numSpotlights; i++) {
    int base = spotlights[i];
    
    // Initialize fixture settings for each spotlight
    DmxMaster.write(base, 0);         // Pan: 0 degrees
    DmxMaster.write(base + 1, 0);     // Tilt: 0 degrees
    DmxMaster.write(base + 3, 0);     // Red intensity
    DmxMaster.write(base + 4, 0);     // Green intensity
    DmxMaster.write(base + 5, 0);     // Blue intensity
    DmxMaster.write(base + 7, 255);   // Dimmer (full brightness)
    DmxMaster.write(base + 8, 0);     // Strobe: off
    DmxMaster.write(base + 9, 0);     // Color macro: off
    DmxMaster.write(base + 10, 0);    // Special functions: none
    DmxMaster.write(base + 11, 0);    // Reserved
  }
}

void loop() {
  for (int colorValue = 0; colorValue <= 255; colorValue++) {
    DmxMaster.write(spotlights[1] + 3, colorValue); // Increase red
    DmxMaster.write(spotlights[1] + 4, 255 - colorValue); // Decrease green

    DmxMaster.write(spotlights[0] + 3, colorValue); // Increase red
    DmxMaster.write(spotlights[0] + 4, 255 - colorValue); // Decrease green
    delay(10);
  }
}
