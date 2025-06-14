/*
    Project: <https://github.com/AngelJMC/4-20ma-wifi-bridge>   
    Copyright (c) 2022 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#include "uinterface.h"
#include "Arduino.h"
#include "config-mng.h"
#include "adc121.h"

/*Create a static freertos timer*/
static TimerHandle_t tmledMode;
static TimerHandle_t tmledState; 


struct ledctrl {
    TimerHandle_t timer;
    uint8_t gpio;
    uint8_t state;
    enum modes mode;
};

static struct ledctrl ledMode;
static struct ledctrl ledState;

enum ledstates{
    AP_MODE = 0,
    STATION_MODE = 1
};

/**
 * @brief Initialize a led control structure
 * @param self, the led control structure
 * @param gpio, gpio identifier when led is connected.
 * @param tmhdl, timer handler to control led bliking. */
static void ledctrl_init( struct ledctrl * self, uint8_t gpio, TimerHandle_t tmhdl ) {
    self->gpio = gpio;
    self->mode = OFF;
    self->state = LED_OFF;
    self->timer = tmhdl;
    digitalWrite(self->gpio, self->state);
}

/**
 * @brief Update the led state
 * @param self, the led control structure
 * @param mode, Setting value of led: OFF, BLINK, ON.*/
static void ledctrl_update( struct ledctrl * self, enum modes mode ) {
    
    if( self->mode == mode ) {
        return;
    }

    self->mode = mode;       
    switch (self->mode) {
        case OFF:
            self->state = LED_OFF;
            xTimerStop( self->timer, pdMS_TO_TICKS( 5 ) );
            digitalWrite(self->gpio, self->state);
            break;
        case BLINK:
            self->state = LED_OFF;
            digitalWrite(self->gpio, self->state);
            xTimerStart( self->timer, pdMS_TO_TICKS( 5 ) );
            break;
        case ON:
            self->state = LED_ON;
            xTimerStop( self->timer, pdMS_TO_TICKS( 5 ) );
            digitalWrite(self->gpio, self->state);
            break;
        default:
            break;
    }
}

/**
 * @brief Callback function used to update the the MODE LED when timer expires.
 * @param xTimer, freertos timer handler */
static void tmledmode_callback( TimerHandle_t xTimer ) {
    ledMode.state = ledMode.state == LOW ? HIGH : LOW;
    digitalWrite(ledMode.gpio, ledMode.state);
}

/**
 * @brief Callback function used to update the STATE LED when timer expires.
 * @param xTimer, freertos timer handler*/
static void tmledstate_callback( TimerHandle_t xTimer ) {
    ledState.state = ledState.state == LOW ? HIGH : LOW;
    digitalWrite(ledState.gpio, ledState.state);
}


void interface_init( void ) {
    
    tmledState = xTimerCreate( "ledState", pdMS_TO_TICKS( 250 ), pdTRUE, NULL, tmledstate_callback );
    ledctrl_init( &ledState, LED2, tmledState );

    tmledMode = xTimerCreate( "ledMode",   pdMS_TO_TICKS( 250 ), pdTRUE, NULL, tmledmode_callback );
    ledctrl_init( &ledMode, LED1, tmledMode );
}

void interface_setMode( enum modes mode ) {
    ledctrl_update( &ledMode, mode );
}

void interface_setState( enum modes mode ) {
    ledctrl_update( &ledState, mode );
}


int board_getadcValue( void ) {
    uint32_t adc_reading = 0;
    enum {
        NO_OF_SAMPLES = 32
    };

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        int16_t raw = 0;
        int err = adc121_getval( &raw );
        if ( err ) {
            Serial.printf("Error reading adc121\n");
            return 0;
        }
        adc_reading += raw;
    }
    
    return adc_reading / NO_OF_SAMPLES;
}

void board_initadc( void ) {
    adc121_init( );
    Serial.println("Init adc...");
}


void factoryreset( void ) {
    Serial.print("Config default mode...");
    config_setdefault(  );
    Serial.println(" Restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP.restart();
}