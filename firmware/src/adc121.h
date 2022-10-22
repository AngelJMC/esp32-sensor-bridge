/*
    Project: <https://github.com/AngelJMC/agriot-sensor-node>   
    Copyright (c) 2020 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#ifndef _SENSORS_H_
#define _SENSORS_H_


#ifdef __cplusplus
extern "C"{
#endif



void adc121_init( void );

int adc121_getval( int16_t* value );

#ifdef __cplusplus
}
#endif

#endif // _SENSORS_H_