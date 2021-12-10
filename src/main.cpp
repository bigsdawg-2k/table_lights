#include <Arduino.h>

#include <FastLED.h>
#include <FastLED_RGBW.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// GPIO for LED on development board
#define PIN_LED_BUILTIN 2

// LED related defines
#define NUM_LEDS 20
#define DATA_PIN 23

// Define the array of LEDs RGBW struct
CRGBW leds[NUM_LEDS];
CRGB *ledsRGB = (CRGB *) &leds[0];

// BLE server setup
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

#define SERVICE_UUID           "242FC2C3-FC6A-4D48-8C86-6DC143A5d684"
#define CHARACTERISTIC_UUID_RX "1730e01c-ec0d-48b9-a8ee-5c6f96c652c7"
#define CHARACTERISTIC_UUID_TX "f5b1ad65-8492-4175-a8ba-da839ee5638f"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

void setup() {

    Serial.begin(115200);

    // Create the BLE Device
    BLEDevice::init("UART Service");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
	    CHARACTERISTIC_UUID_TX,
		BLECharacteristic::PROPERTY_NOTIFY
		);
                      
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
	    CHARACTERISTIC_UUID_RX,
		BLECharacteristic::PROPERTY_WRITE
		);

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");

    // put your setup code here, to run once:
    pinMode(PIN_LED_BUILTIN, OUTPUT);

    // Setup LED strip for FastLED with RGBW
    // FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // This is the unmodified call for reference
    FastLED.addLeds<WS2812B, DATA_PIN, RGB>(ledsRGB, getRGBWsize(NUM_LEDS));

}

void loop() {
  
    if (deviceConnected) {
        //pTxCharacteristic->setValue(&txValue, 1);
        uint8_t a = 83;
        pTxCharacteristic->setValue(&a, 1);
        pTxCharacteristic->notify();
        txValue++;
		delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    
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