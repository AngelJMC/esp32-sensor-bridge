/*
    Project: <https://github.com/AngelJMC/4-20ma-wifi-bridge>   
    Copyright (c) 2022 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#include "Arduino.h"
#include "webserver.h"
#include "mqtt_task.h"
#include "sensor-task.h"
#include "config-mng.h"
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include "uinterface.h"

#define CONFIG_ASYNC_TCP_RUNNING_CORE 

static TimerHandle_t tmswitch;

static int timerstate = 0;


static void switch_callback( TimerHandle_t xTimer ) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (timerstate == 0) {
        if( digitalRead( SWITCH ) == LOW ) {
            if( xTimerStopFromISR( tmswitch, &xHigherPriorityTaskWoken ) != pdPASS ) {
                /* The stop command was not executed successfully.  Take appropriate
                action here. */
            }
            /*Toggle configuration mode*/
            ctrl_isConfigModeEnable() ? ctrl_exitConfigMode( ) : ctrl_enterConfigMode( );
        } 
        else {
            if( xTimerChangePeriodFromISR( tmswitch,
                                        pdMS_TO_TICKS( 4000 ),
                                        &xHigherPriorityTaskWoken ) != pdPASS ){
            /* The command to change the timers period was not executed
            successfully.  Take appropriate action here. */
            } 
            timerstate = 1;
        }
    } 
    else {
        if( xTimerStopFromISR( tmswitch, &xHigherPriorityTaskWoken ) != pdPASS ) {
            /* The stop command was not executed successfully.  Take appropriate
            action here. */
        }
        if( digitalRead( SWITCH ) == HIGH) {
            factoryreset( );
        }
    }
}


void Ext_INT1_ISR( void ) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if( xTimerChangePeriodFromISR( tmswitch,
                                   pdMS_TO_TICKS( 1000 ),
                                   &xHigherPriorityTaskWoken ) != pdPASS ){
        /* The command to change the timers period was not executed
        successfully.  Take appropriate action here. */
    } 
    
    if( xTimerStartFromISR( tmswitch, &xHigherPriorityTaskWoken ) != pdPASS ) {
        /* The start command was not executed successfully.  Take appropriate
        action here. */
    }

    timerstate = 0;
}




void setup() {
    //vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.begin(115200);
    
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(SWITCH, INPUT);

    digitalWrite(LED1, LED_OFF);
    digitalWrite(LED2, LED_OFF);
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);

    attachInterrupt(SWITCH, Ext_INT1_ISR, RISING);
    config_load(  );

    sensor_init( );

    //########################  reading config file ########################################
    if (!SPIFFS.begin()) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    
    tmswitch = xTimerCreate( "tmSwitch",   pdMS_TO_TICKS( 250 ), pdTRUE, NULL, switch_callback );
    interface_init( );
    // Now set up two Tasks to run independently.
    xTaskCreate( webserver_task , "webserver-task",  1024*10  ,NULL  ,  2,  NULL );
    xTaskCreate( ctrl_task ,      "ctrl-task",       1024*3   ,NULL  ,  1,  NULL );
    xTaskCreate( sensor_task,     "sensor-task",     1024*2   ,NULL  ,  1,  NULL );

}


void loop() { 

}