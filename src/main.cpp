#include <Arduino.h>
#include <FastLED.h>
#include <FastLED_RGBW.h>

// GPIO for LED on development board
#define PIN_LED_BUILTIN 2

// LED related defines
#define NUM_LEDS 20
#define DATA_PIN 23

// Define the array of LEDs RGBW struct
CRGBW leds[NUM_LEDS];
CRGB *ledsRGB = (CRGB *) &leds[0];

void setup() {
  
    // put your setup code here, to run once:
    pinMode(PIN_LED_BUILTIN, OUTPUT);

    // Setup LED strip for FastLED with RGBW
    // FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // This is the unmodified call for reference
    FastLED.addLeds<WS2812B, DATA_PIN, RGB>(ledsRGB, getRGBWsize(NUM_LEDS));

}

void loop() {
  
    // put your main code here, to run repeatedly:
    // Builtin LED is left in as a sanity check
    for (int dot = 0; dot < NUM_LEDS; dot++) { 
    
        leds[dot] = CRGB::Blue;
        FastLED.show();
    
        // clear this led for the next time around the loop
        leds[dot] = CRGB::Black;
    
        // Builtin LED is left in as a sanity check
        if (dot % 2 == 0) {
            digitalWrite(PIN_LED_BUILTIN, HIGH);
        } else {
            digitalWrite(PIN_LED_BUILTIN, LOW);
        }
    
        delay(100);

    }

    for (int idx = 0; idx < NUM_LEDS; ++idx) {
        leds[idx] = CRGB::Black;
    }
    
    FastLED.show();
    digitalWrite(PIN_LED_BUILTIN, LOW);
  
    delay(100);
 
}