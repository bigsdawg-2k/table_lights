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

// LED FSM State
#define FSM_STATE_LED_RESET     0
#define FSM_STATE_LED_IDLE      1
#define FSM_STATE_LED_STATIC    2
int fsmState_led = FSM_STATE_LED_RESET;

// Define the array of LEDs RGBW struct
CRGBW leds[NUM_LEDS];
CRGB *ledsRGB = (CRGB *) &leds[0];

// Serial over bluetooth state
BluetoothSerial SerialBT;

// Buffer that will be used to store commands received over BT
CmdBuffer* cmdBuf = new CmdBuffer(NUM_BYTES_SERIAL_BT_BUFF);
SemaphoreHandle_t xBinSem_cmdBuf;

// Miscellaneous buffer for string processing
char strBuffer [1024];


/**********************************
/* Bluetooth callbacks            *  
/**********************************/

/* Bluetooth Event Callback
 * Handles events from BT:
 *      ESP_SPP_SRV_OPEN_EVT
 *      ESP_SPP_CLOSE_EVT
 *      ESP_SPP_DATA_IND_EVT
 *          - Received data - add it to the command buffer
 * Arguments:
 *      event
 *      parameter
 * Returns: 
 *      Nothing
 */
void BTEventCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) 
    {
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
            if (cmdBuf->getFreeSerBuf() >= param->data_ind.len)
            {
                // TODO replace hardcoded 10ms delay
                if (xSemaphoreTake(xBinSem_cmdBuf, 10 / portTICK_PERIOD_MS) == pdTRUE)
                {
                    cmdBuf->write((char *)param->data_ind.data, param->data_ind.len);
                    xSemaphoreGive(xBinSem_cmdBuf);
                } 
            } 
            else
            {
                // TODO: Something useful here
                Serial.println("Buffer overflow detected in serial buffer");
            }
            break;
        default:
            Serial.println("Other event!");
    }
}


/**********************************
 * Application tasks              *  
 **********************************/

/* Update the LED state machine when there is a command in the command buffer 
 *
 * Arguments:
 *      None
 * Returns: 
 *      Nothing
 */ 
void appTask_updateLedFsmState (void * parameter)
{
    // TODO: Placeholder 'code'
    for(;;)
    {
        if (cmdBuf->getOccupied() > 0 && 
            xSemaphoreTake(xBinSem_cmdBuf, 10 / portTICK_PERIOD_MS)) // TODO remove hardcoded 10ms delay
        {
            cmdBuf->
            xSemaphoreGive(xBinSem_cmdBuf);
        }
        Serial.println("Nothing to report...");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // TODO remove hardcoded 1s delay
    }
    
    // Delete a task when finished 
    vTaskDelete( NULL );
   
}

/* Update LED state based on FSM state
 *
 * Arguments:
 *      None
 * Returns: 
 *      Nothing
 */ 
void appTask_updateLeds (void * parameter)
{
    // TODO: Placeholder 'code'
    for(;;)
    {
        Serial.println("this is another Task");
        delay(1000);
    }
    
    // Delete a task when finished 
    vTaskDelete( NULL );
   
}


// This gets set as the default handler, and gets called when no other command matches.
void Unrecognized (const char *command) 
{
    Serial.print("Unrecognized command: ");
    Serial.println(command);
}

void UpdateLedPattern (int pattern) {
    
    switch ( pattern )
    {
        case 0:
        case 1:
            fsmState_led = pattern;
            break;
        default:
            sprintf(strBuffer, "Unrecognized state: %d", pattern);
            Serial.println(strBuffer);
    }
    
}

/* Setup function
 *
 * Arguments:
 *      None
 * Returns: 
 *      Nothing
 */
void setup() 
{

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

    // Semaphores
    Serial.write("Creating semaphore...");
    if (xBinSem_cmdBuf == NULL)
    {
        xBinSem_cmdBuf = xSemaphoreCreateBinary();
        if (xBinSem_cmdBuf == NULL)
        {
            Serial.write("Semaphore is null.");
        }
        else
        {
            Serial.write("Created semaphore!");
        }
        
    }
    
    // RTOS Tasks
    xTaskCreate(
        appTask_updateLedFsmState,  // Function name
        "Update LED FSM State",     // Task name
        10000,                      // Stack size
        NULL,                       // Task parameter
        1,                          // Task priority
        NULL);                      // Task handle

    xTaskCreate(
        appTask_updateLeds,         // Function name
        "Update LEDs",              // Task name
        10000,                      // Stack size
        NULL,                       // Task parameter
        1,                          // Task priority
        NULL);                      // Task handle    

    // Physical LED on development kit
    pinMode(PIN_LED_BUILTIN, OUTPUT);

    // Setup LED strip for FastLED with RGBW
    // FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // This is the unmodified call for reference
    FastLED.addLeds<WS2812B, DATA_PIN, RGB>(ledsRGB, getRGBWsize(NUM_LEDS));
    
}

/* Forever loop that is invoked by ESP32 loopTask
 *
 * Arguments:
 *      None
 * Returns: 
 *      Nothing
 */
void loop() 
{
    
    int i;
    
    // put your main code here, to run repeatedly:
    // Builtin LED is left in as a sanity check
    for (i = 0; i < NUM_LEDS; i++) 
    { 
    
        leds[i] = CRGB::Blue;
        FastLED.show();
    
        // clear this led for the next time around the loop
        leds[i] = CRGB::Black;
    
        // Builtin LED is left in as a sanity check
        if (i % 2 == 0) {
            digitalWrite(PIN_LED_BUILTIN, HIGH);
        } else {
            digitalWrite(PIN_LED_BUILTIN, LOW);
        }
    
        delay(100);

    }

    for (i = 0; i < NUM_LEDS; ++i) 
    {
        leds[i] = CRGB::Black;
    }
    
    FastLED.show();
    digitalWrite(PIN_LED_BUILTIN, LOW);
  
    delay(100);

    // See if anything new is in the serial buffer
    //if (lastSerBufSize == rbSerBuf->getOccupied()){
    //    numCmds = ;
    //}

    if (cmdBuf->getOccupied() > 0) 
    {
        // TODO How to make sure there are no conflicts when reading out the buffer
        Serial.print("Buffer: ");
        //cmdBuf->readToEnd((uint8_t *)strBuffer);
        // TODO: replace with get command
        Serial.println(strBuffer);
    }
    
    // Display the current LED state
    sprintf(strBuffer, "LED Pattern: %d", ::fsmState_led);
    Serial.println(strBuffer);
 
}

