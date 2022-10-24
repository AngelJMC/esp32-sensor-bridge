
#include "Arduino.h"

#include <Wire.h>
#include "adc121.h"


enum {
    REG_ADDR_RESULT   = 0x00,
    REG_ADDR_ALERT    = 0x01,
    REG_ADDR_CONFIG   = 0x02,
    REG_ADDR_LIMITL   = 0x03,
    REG_ADDR_LIMITH   = 0x04,
    REG_ADDR_HYST     = 0x05,
    REG_ADDR_CONVL    = 0x06,
    REG_ADDR_CONVH    = 0x07,

    ADDR_ADC121       = 0x51,
};

static unsigned int adcval;    



int adc121_getval( int16_t* value ) {
    int16_t adcval = 0;
    Wire.beginTransmission( ADDR_ADC121 );        // transmit to device
    Wire.write( REG_ADDR_RESULT );                // get reuslt
    Wire.endTransmission();
    Wire.requestFrom( ADDR_ADC121, 2 );   
    if( Wire.available() == 2 ) {
        adcval = (Wire.read()&0x0f)<<8;
        adcval |= Wire.read();
        *value = adcval;
        return 0;
    }
    return -1;
}


void adc121_init( void ) {

    Wire.begin();
    Wire.beginTransmission( ADDR_ADC121 );        // transmit to device
    Wire.write( REG_ADDR_CONFIG );                // Configuration Register
    Wire.write( 0x02 );  //0x20  //0x10 works
    Wire.endTransmission(); 

}



