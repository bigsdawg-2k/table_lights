#include <Arduino.h>

#include <FastLED.h>
#include <FastLED_RGBW.h>

#include <BluetoothSerial.h>

#include <cmdBuffer.h>

// GPIO for LED on development board
#define PIN_LED_BUILTIN 2

// LED related defines
#define NUM_LEDS 20
#define DATA_PIN 23

#define NUM_BYTES_COMMAND_BUFF 1024
#define NUM_BYTES_SERIAL_BT_BUFF 1024

// LED pattern
// 0: off
// 1: static on
int ledPattern = 0;

// Define the array of LEDs RGBW struct
CRGBW leds[NUM_LEDS];
CRGB *ledsRGB = (CRGB *) &leds[0];

// Serial over bluetooth state
BluetoothSerial SerialBT;

// Buffer that will be used to store commands received over BT
CmdBuffer* cmdBuf = new CmdBuffer(NUM_BYTES_SERIAL_BT_BUFF);

// Miscellaneous buffer for string processing
char strBuffer [1024];

void BTEventCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
    switch (event) {
    case ESP_SPP_SRV_OPEN_EVT:
        Serial.println("Client Connected");
        break;
    case ESP_SPP_CLOSE_EVT:
        Serial.println("Client disconnected");
        break;
    case ESP_SPP_DATA_IND_EVT:
        Serial.println("Received data: ");
        
        // TODO sanitize buffer room
        //std::copy(param->data_ind.data, param->data_ind.data + param->data_ind.len, commandBuffer + commandBufferBytes);
        //commandBufferBytes += param->data_ind.len;
        if(cmdBuf->getFreeSerBuf() >= param->data_ind.len){
            cmdBuf->write((char *)param->data_ind.data, param->data_ind.len);
        } 
        else {
            // Eventually do something more useful here
            Serial.println("Buffer overflow detected in serial buffer");
        }
        break;
    default:
        Serial.println("Other event!");
    }
}

// This gets set as the default handler, and gets called when no other command matches.
void Unrecognized(const char *command) {
    Serial.print("Unrecognized command: ");
    Serial.println(command);
}

void UpdateLedPattern (int pattern) {
    
    switch ( pattern )
    {
    case 0:
    case 1:
        ledPattern = pattern;
        break;
    default:
        sprintf(strBuffer, "Unrecognized state: %d", pattern);
        Serial.println(strBuffer);
    }
    
}

void setup() {

    // Serial over USB settings
    Serial.begin(115200);

    // Serial over BT settings
    SerialBT.register_callback(BTEventCallback);
    if (!SerialBT.begin("Table Lights")) {
        Serial.println("An error occurred initializing Bluetooth");
    }
    else {
        Serial.println("Bluetooth initialized");
    }

    // Physical LED on development kit
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

    // See if anything new is in the serial buffer
    //if (lastSerBufSize == rbSerBuf->getOccupied()){
    //    numCmds = ;
    //}

    if (cmdBuf->getOccupied() > 0) {
        // TODO How to make sure there are no conflicts when reading out the buffer
        Serial.print("Buffer: ");
        //cmdBuf->readToEnd((uint8_t *)strBuffer);
        // TODO: replace with get command
        Serial.println(strBuffer);
    }
    
    // Display the current LED state
    sprintf(strBuffer, "LED Pattern: %d", ::ledPattern);
    Serial.println(strBuffer);
 
}

