#include "sensor-task.h"
#include <ld2410.h>
#include "freertos/queue.h"


#define RXD2 16 // 8 
#define TXD2 17 // 9

ld2410 radar;

uint32_t lastReading = 0;


uint32_t pos = 0;
char buffer1[128];
char serialBuffer[256];

void filldataFrame( struct dataframe* data ) {

    data->detectionDistance        = radar.detectionDistance();
    data->stationaryTargetDistance = radar.stationaryTargetDistance();
    data->stationaryTargetEnergy   = radar.stationaryTargetEnergy();
    data->movingTargetDistance     = radar.movingTargetDistance();
    data->movingTargetEnergy       = radar.movingTargetEnergy();
    data->engRataingData           = radar.engRetainDataValue();
  
    for (int x = 0; x < LD2410_MAX_GATES; ++x) {
        data->engMovingDistanceGateEnergy[x] = radar.engMovingDistanceGateEnergy(x);
        data->engStaticDistanceGateEnergy[x] = radar.engStaticDistanceGateEnergy(x);
    }
}


String dataStructureToCsv( struct dataframe const* data) {
    pos = snprintf(serialBuffer, sizeof(serialBuffer),
                    "%d,%d,%d,%d,%d,%d",
                    data->stationaryTargetDistance, data->stationaryTargetEnergy,
                    data->movingTargetDistance, data->movingTargetEnergy,
                    data->detectionDistance, data->engRataingData );

    for (int x = 0; x < LD2410_MAX_GATES; ++x) {
        int len = snprintf(buffer1, sizeof(buffer1), ",%d", data->engMovingDistanceGateEnergy[x]);
        strcat(serialBuffer, buffer1);
        pos += len;
    }
    for (int x = 0; x < LD2410_MAX_GATES; ++x) {
        int len = snprintf(buffer1, sizeof(buffer1), ",%d",data->engStaticDistanceGateEnergy[x]);
        strcat(serialBuffer, buffer1);
        pos += len;
    }

    return String(serialBuffer);
}

static QueueHandle_t queue;
bool newdata = false;
static struct dataframe dataf;

bool waitnewData( struct dataframe *data) {

    if( xQueueReceive(queue, data, (TickType_t)pdMS_TO_TICKS(250)) ) {
        return true;
    }
    return false;
}

void sensor_init( void ) {
    int sizeitem = sizeof( struct dataframe );
    queue = xQueueCreate( 5, sizeitem );
    if (queue == 0) {
        Serial.printf("Failed to create queue= %p\n", queue);
    }

}

void sensor_task( void * parameter ) {


    // start path to LD2410
    // radar.debug(Serial);  // enable debug output to console
    Serial2.begin(256000, SERIAL_8N1, RXD2, TXD2); // UART for monitoring the radar rx, tx
    // Start LD2410 Sensor
    if (radar.begin(Serial2)) {
        Serial.println(F("Sensor Initialized..."));
        delay(5000);
        radar.requestStartEngineeringMode();
    } else {
        Serial.println(F(" Sensor was not connected"));
    }
    
    for(;;){
        radar.ld2410_loop();
        if(radar.isConnected() && millis() - lastReading > 100) {  //Report every 1000ms
            filldataFrame( &dataf ); 
            newdata = true;
            int status = xQueueSend( queue, &dataf, pdMS_TO_TICKS(250) );
            if ( !status ) 
                Serial.println( "Failing queue");
            
            lastReading = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

}
