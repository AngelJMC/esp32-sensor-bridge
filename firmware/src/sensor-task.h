
#ifndef __SENSOR_TASK__
#define __SENSOR_TASK__

#include "ld2410.h"

struct dataframe {
    uint16_t detectionDistance;
    uint16_t stationaryTargetDistance;
    uint8_t  stationaryTargetEnergy;
    uint16_t movingTargetDistance;
    uint8_t  movingTargetEnergy;
    uint16_t engRataingData;
    uint8_t  engMovingDistanceGateEnergy[LD2410_MAX_GATES];
    uint8_t  engStaticDistanceGateEnergy[LD2410_MAX_GATES];
};


/**
 * @brief Freertos task to manage the WIFI and MQTT connections.
 * @param parameter */
void sensor_task( void * parameter );

bool waitnewData( struct dataframe *data);

String dataStructureToCsv( struct dataframe const* data);

void sensor_init( void );

#endif //__SENSOR_TASK__