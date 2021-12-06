#include <Arduino.h>
#include <FastLED.h>

// GPIO for LED on development board
#define PIN_LED_BUILTIN 2

// LED related defines
#define NUM_LEDS 10
#define PIN_DATA 23

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() {
  
  // put your setup code here, to run once:
  pinMode(PIN_LED_BUILTIN, OUTPUT);

  // Setup LED strip
  FastLED.addLeds<SK6812, PIN_DATA, GRB>(leds, NUM_LEDS);  // GRB ordering is typical

}

void loop() {
  
  // put your main code here, to run repeatedly:
  // Builtin LED is left in as a sanity check
  digitalWrite(PIN_LED_BUILTIN, HIGH);
  for (int idx = 0; idx < NUM_LEDS; ++idx) {
    leds[idx] = CRGB::Red;
  }
  FastLED.show();

  delay(1000);

  digitalWrite(PIN_LED_BUILTIN, LOW);
  for (int idx = 0; idx < NUM_LEDS; ++idx) {
    leds[idx] = CRGB::Black;
  }
  FastLED.show();
  
  delay(1000);
 
}