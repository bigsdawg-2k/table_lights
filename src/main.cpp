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

// Task and OS Resource Rest Times
#define TASK_WAIT_LEDFSM_ms     1000
#define TASK_WAIT_EXEC_CMD_ms   10
#define Q_WAIT_LEDFSM_WAIT_ms   10

// Module Targest
#define MODULE_LED              1

// LED FSM related
#define LEDFSM_STATE_RESET      0
#define LEDFSM_STATE_IDLE       1
#define LEDFSM_STATE_COMMAND    2
#define LEDFSM_STATE_UPDATE     3
QueueHandle_t   xQLedfsm;

// LED Module Instructions
#define LEDMOD_INST_OFF         0
#define LEDMOD_INST_ON          1
#define LEDMOD_INST_SET_PATTERN 2

// LED states
#define LEDS_UPDATE_PERIOD_ms   500
#define LEDS_PATTERN_OFF        0
#define LEDS_PATTERN_STATIC_ON  1
#define LEDS_PATTERN_BLINK      2
int gLedsState = 0;

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


void printCommand(cmdItem cmd)
{
    Serial.print("Received command: ");
    Serial.print(cmd.moduleTarget);
    Serial.print(", ");
    Serial.print(cmd.instruction);
    Serial.print(", ");
    Serial.print(cmd.vArgLen);
    for (int idx = 0; idx < cmd.vArgLen; idx++)
    {
        if (idx == 0) {Serial.print(":");}
        Serial.print(cmd.vArg[idx]);
        Serial.print(".");
    }
    Serial.print("\n");
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
    
    while( pdTRUE )
    {
        if (cmdBuf->getOccupied() > 0 && 
            xSemaphoreTake( xBinSem_cmdBuf, 10 / portTICK_PERIOD_MS )) // TODO remove hardcoded 10ms delay
        {
            tmpInt = cmdBuf->readCmd( &tmpCmd );
            xSemaphoreGive( xBinSem_cmdBuf );
        }

        if (tmpInt > 0)
        {
            printCommand(tmpCmd);
            tmpInt = 0;

            switch (tmpCmd.moduleTarget)
            {
            
            // LED module
            case MODULE_LED: 
                Serial.printf("Received command for LED module\n");
                if( xQueueSend( xQLedfsm, (void *) &tmpCmd, (TickType_t) Q_WAIT_LEDFSM_WAIT_ms / portTICK_PERIOD_MS ) != pdTRUE )
                {
                    // TODO: Handle failed post command to LED FSM queue
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

        vTaskDelay(TASK_WAIT_EXEC_CMD_ms / portTICK_PERIOD_MS);
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
void xTask_fsmLeds (void * parameter)
{
    
    // State variables
    int fsmledState = LEDFSM_STATE_RESET;

    // LED variables
    bool    ledsEnabled;
    int     ledsPattern;
    int     ledsBlink_ms;
    cmdItem cmdFromQ;

    // Managing the updated period tracking
    TickType_t  xTicksToLedUpdate = LEDS_UPDATE_PERIOD_ms / portTICK_PERIOD_MS;
    TimeOut_t   xLedsUpdateTO;
    
    while( true )
    {
        
        switch( fsmledState )
        {
        case LEDFSM_STATE_RESET:
            
            // Start the LED update timeout
            ledsEnabled = pdFALSE;
            ledsPattern = LEDS_PATTERN_OFF;
            ledsBlink_ms = 1000;
            fsmledState = LEDFSM_STATE_UPDATE;
            break;
        
        case LEDFSM_STATE_IDLE:

            // Check command queue
            if( xQueueReceive( xQLedfsm, &cmdFromQ, 0) == pdTRUE )
            {
                fsmledState = LEDFSM_STATE_COMMAND;
                break;
            } 
            // Due for an update
            else if( ledsEnabled && xTaskCheckForTimeOut( &xLedsUpdateTO, &xTicksToLedUpdate ) == pdTRUE )
            {
                fsmledState = LEDFSM_STATE_UPDATE;
                break;
            }
            // Due for an update soon
            else if(  ledsEnabled && xTicksToLedUpdate < TASK_WAIT_LEDFSM_ms / portTICK_PERIOD_MS )
            {
                Serial.printf("Waiting shortly (%d)\n", xTicksToLedUpdate);
                vTaskDelay( xTicksToLedUpdate );
            }
            // Nothing to do, just delay
            else
            {
                Serial.printf("Waiting normally\n");
                vTaskDelay( TASK_WAIT_LEDFSM_ms / portTICK_PERIOD_MS );
            }
            
            break;
        
        // Handle command for the LED module from the queue
        case LEDFSM_STATE_COMMAND:

            printCommand(cmdFromQ);

            switch( cmdFromQ.instruction )
            {
            case LEDMOD_INST_OFF:
                Serial.printf("Turning LED(s) off.\n");
                ledsEnabled = pdFALSE;
                break;

            case LEDMOD_INST_ON:
                Serial.printf("Turning LED(s) on.\n");
                ledsEnabled = pdTRUE;
                break;

            case LEDMOD_INST_SET_PATTERN:
                Serial.printf( "Setting LED(s) to (%d)\n", cmdFromQ.vArg[0] );
                if( cmdFromQ.vArgLen > 0 )
                {
                    ledsPattern = cmdFromQ.vArg[0];
                }
                break;

            default:
                Serial.printf("Unrecognized instruction for LED module (%d)\n", cmdFromQ.instruction);
                break;
            }
            
            // Get rid of any arguments now that they have been used
            if( cmdFromQ.vArgLen > 0 )
            {
                free( cmdFromQ.vArg );
                cmdFromQ.vArgLen = 0;
            }

            fsmledState = LEDFSM_STATE_IDLE;
            break;
        
        case LEDFSM_STATE_UPDATE:

            Serial.printf("Updating LEDs\n");
            switch( ledsPattern )
            {
            case LEDS_PATTERN_OFF:
                digitalWrite(PIN_LEDS_BUILTIN, LOW);
                break;
            case LEDS_PATTERN_STATIC_ON:
                digitalWrite(PIN_LEDS_BUILTIN, HIGH);
                break;
            case LEDS_PATTERN_BLINK:
                break;
            default:
                break;
            }
            
            // Update complete, reset the update timeout
            vTaskSetTimeOutState( &xLedsUpdateTO );
            xTicksToLedUpdate = LEDS_UPDATE_PERIOD_ms / portTICK_PERIOD_MS;
            fsmledState = LEDFSM_STATE_IDLE;
            break;
        
        default:
            break;
        }

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

    // Queues
    xQLedfsm = xQueueCreate( 10, sizeof(cmdItem) );
    if(xQLedfsm == 0)
    {
        // TODO: Handle failed to create queue
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
        xTask_fsmLeds,             // Function name
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
 
}

