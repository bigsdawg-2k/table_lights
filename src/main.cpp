#include <Arduino.h>

#include <FastLED.h>
#include <FastLED_RGBW.h>

#include <BluetoothSerial.h>

#include <cmdBuffer.h>

// GPIO for LED on development board
#define PIN_LEDS_BUILTIN 2

// LED related defines
#define NUM_LEDS 20
#define DATA_PIN 23

#define NUM_BYTES_COMMAND_BUFF 1024
#define NUM_BYTES_SERIAL_BT_BUFF 1024

// LED FSM related
#define LEDFSM_STATE_RESET     0
#define LEDFSM_STATE_IDLE      1
#define LEDFSM_STATE_UPDATE    2
#define LEDFSM_REST_TIME_ms    10
#define LEDFSM_REST_TIME_ticks (LEDFSM_REST_TIME_ms / portTICK_PERIOD_MS)


// LED states
#define LEDS_UPDATE_PERIOD_ms   50
#define LEDS_UPDATE_PERIOD_ticks (LEDS_UPDATE_PERIOD_ms / portTICK_PERIOD_MS)
#define LEDS_STATE_OFF          pdFALSE
#define LEDS_STATE_ON           pdTRUE
#define LEDS_PATTERN_OFF        0
#define LEDS_PATTERN_STATIC_ON  1
SemaphoreHandle_t xBinSem_ledsState;
int gLedsState = 0;
// TODO: 20220113 - REPLACE THIS SEMAPHORE BASED MESSAGING WITH QUEUE INTO LED_FSM

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
            if (cmdBuf->getFree() >= param->data_ind.len)
            {
                // TODO replace hardcoded 10ms delay
                if (xSemaphoreTake(xBinSem_cmdBuf, 10 / portTICK_PERIOD_MS) == pdTRUE)
                {
                    cmdBuf->writeCmdMsg((char *)param->data_ind.data, param->data_ind.len);
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
void xTask_execCmd (void * parameter)
{
    
    static cmdItem tmpCmd;
    static int tmpInt;

    // TODO: Placeholder 'code'
    for(;;)
    {
        if (cmdBuf->getOccupied() > 0 && 
            xSemaphoreTake(xBinSem_cmdBuf, 10 / portTICK_PERIOD_MS)) // TODO remove hardcoded 10ms delay
        {
            tmpInt = cmdBuf->readCmd(&tmpCmd);
            xSemaphoreGive(xBinSem_cmdBuf);
        }

        if (tmpInt > 0)
        {
            Serial.print("Received command: ");
            Serial.print(tmpCmd.moduleTarget);
            Serial.print(", ");
            Serial.print(tmpCmd.instruction);
            Serial.print(", ");
            Serial.print(tmpCmd.vArgLen);
            for (tmpInt = 0; tmpInt < tmpCmd.vArgLen; tmpInt++)
            {
                if (tmpInt == 0) {Serial.print(":");}
                Serial.print(tmpCmd.vArg[tmpInt]);
                Serial.print(".");
            }
            Serial.println("");
            tmpInt = 0;

            switch (tmpCmd.moduleTarget)
            {
            case 1: // LED module
                Serial.printf("Received command for LED module\n");
                switch (tmpCmd.instruction)
                {
                case 0: // Set LED state 'off'
                    if( xSemaphoreTake( xBinSem_ledsState, 10 / portTICK_PERIOD_MS )) // TODO remove hardcoded 10ms delay
                        { ledsState = LEDS_STATE_OFF; }
                    Serial.println("Turning LED(s) off.\n");
                    xSemaphoreGive(xBinSem_ledsState);
                    break;
                case 1: // Set LED state to arg
                    if (tmpCmd.vArgLen == 1 && 
                        xSemaphoreTake(xBinSem_ledsState, 10 / portTICK_PERIOD_MS)) // TODO remove hardcoded 10ms delay
                        { ledsState = tmpCmd.vArg[0]; }
                        Serial.printf("Setting LED(s) state to (%d)\n", tmpCmd.vArg[0]);
                    xSemaphoreGive(xBinSem_ledsState);
                    break;
                case 2: // Set LED pattern to arg
                    // TODO add this state
                    break;
                default:
                    Serial.printf("Unrecognized instruction for LED module (%d)\n", tmpCmd.instruction);    
                    break;
                }
                break;
            default:
                Serial.printf("Unrecognized target module (%d)\n", tmpCmd.moduleTarget);
                break;
            }
        }
        else
        {
            // Serial.println("Nothing to report...");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // TODO remove hardcoded 1s delay
    }
    
    // Delete a task when finished 
    vTaskDelete( NULL );
   
}

/* Finite state machine for LED module
 *
 * Arguments:
 *      None
 * Returns: 
 *      Nothing
 */ 
void xTask_fsdmLeds (void * parameter)
{
    
    // State variables
    int ledFsmState = LEDFSM_STATE_RESET;

    // LED variables
    bool ledsEnabled = LEDS_STATE_OFF;
    int ledsPattern = LEDS_PATTERN_OFF;

    // Managing the updated period tracking
    TickType_t  xTicksToUpdate = LEDS_UPDATE_PERIOD_ticks;
    TimeOut_t   xUpdateTO;
    
    while (true)
    {
        switch (ledFsmState)
        {
        
        case LEDFSM_STATE_RESET:
            
            // Start the LED update timeout
            vTaskSetTimeOutState( &xUpdateTO );

            ledsEnabled = LEDS_STATE_OFF;
            ledsPattern = LEDS_PATTERN_OFF;

            ledFsmState = LEDFSM_STATE_UPDATE;
            break;
        
        case LEDFSM_STATE_IDLE:

            // Due for an update
            if( xTaskCheckForTimeOut( &xUpdateTO, &xTicksToUpdate ) == pdTRUE )
            {
                ledFsmState = LEDFSM_STATE_UPDATE;
                break;
            }
            // No update, delay
            else if( xTicksToUpdate < LEDFSM_REST_TIME_ticks )
            {
                vTaskDelay( xTicksToUpdate );
            }
            else
            {
                vTaskDelay( LEDFSM_REST_TIME_ticks );
            }
            break;
        
        case LEDFSM_STATE_UPDATE:
            
            // Update complete, reset the update timeout
            vTaskSetTimeOutState( &xUpdateTO );
            
            ledFsmState = LEDFSM_STATE_IDLE;
            break;
        
        default:
            break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // TODO remove hardcoded 0.01s delay 
    }
    
    // Delete a task when finished 
    vTaskDelete( NULL );
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
    if (xBinSem_cmdBuf == NULL)
    {
        xBinSem_cmdBuf = xSemaphoreCreateBinary();
        if (xBinSem_cmdBuf != NULL) { xSemaphoreGive(xBinSem_cmdBuf); }
    }

    if (xBinSem_ledsState == NULL)
    {
        xBinSem_ledsState = xSemaphoreCreateBinary();
        if (xBinSem_ledsState != NULL) { xSemaphoreGive(xBinSem_ledsState); }
    }

    // RTOS Tasks
    xTaskCreate(
        xTask_execCmd,              // Function name
        "Execute Command",          // Task name
        10000,                      // Stack size
        NULL,                       // Task parameter
        1,                          // Task priority
        NULL);                      // Task handle

    xTaskCreate(
        xTask_fsdmLeds,             // Function name
        "LED FSM",                  // Task name
        10000,                      // Stack size
        NULL,                       // Task parameter
        1,                          // Task priority
        NULL);                      // Task handle    

    // Physical LED on development kit
    pinMode(PIN_LEDS_BUILTIN, OUTPUT);

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
        
    }

    for (i = 0; i < NUM_LEDS; ++i) 
    {
        leds[i] = CRGB::Black;
    }
    
    FastLED.show();
    
    delay(100);

    // Display the current LED state
    sprintf(strBuffer, "LED Pattern: %d", ::ledsState);
    Serial.println(strBuffer);
 
}

