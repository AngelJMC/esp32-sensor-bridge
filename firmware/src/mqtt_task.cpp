/*
    Project: <https://github.com/AngelJMC/4-20ma-wifi-bridge>   
    Copyright (c) 2022 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#include <WiFi.h>
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include "webserver.h"
#include "config-mng.h"
#include "mqtt_task.h"
#include "time.h"
#include "uinterface.h"
#include "json-maker/json-maker.h"

#include "Wire.h"
#include "SHTSensor.h"
#include "sensor-task.h"

#include <AsyncUDP.h>


enum {
    verbose = 1
};

/*Calibration equation*/
struct caleq {
    double m;
    double b;
};

WiFiClient wifiClient;
PubSubClient client(wifiClient);

SHTSensor sht;

AsyncUDP udp;


/*Create a static freertos timer*/
static TimerHandle_t tmPubMeasurement;
static TimerHandle_t tmPubStatus;
static TimerHandle_t tmPubInfo;
static bool isServerActive = false;

/* Declare a variable to hold the created event group. */
static EventGroupHandle_t events;
static struct service_config scfg;
static struct caleq eq = { .m = 1.0, .b = 0.0 };

static struct ctrl_status {
    int relay1 = LOW;
    int relay2 = LOW;
    int enableTemp = false;
} ctrlStatus;

enum {
    JSON_TX_SIZE = 512
};

static char payload_json[JSON_TX_SIZE];

enum flags {
    START_AP_WIFI = 1 << 0,
    CONNECT_WIFI  = 1 << 1,
    CONNECT_MQTT  = 1 << 2,
    PUB_INFO      = 1 << 3,
    PUB_STATUS    = 1 << 4,
    PUB_MEASURES  = 1 << 5
};

struct sensors{ 
    char const* id; 
    int (*getsample)( struct sensors const*, double* ); 
    char const* unit;
    struct {
        double min;
        double max;
    } const range;
};

static double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    
    val = val < in_min ? in_min : val;
    val = val > in_max ? in_max : val;
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int get_sht3x_temperature( struct sensors const* self, double* value ) {

    int st = sht.readSample();
    if ( st == 0 )
        return -1;

    *value = sht.getTemperature();
    return 0;
}

static int get_sht3x_humidity( struct sensors const* self, double* value ) {
    int st = sht.readSample();
    if ( st == 0 )
        return -1;

    *value = sht.getHumidity();
    return 0;
}

static int get_gas_sensor( struct sensors const* self, double* value ){
    double raw = board_getadcValue();
    double cali =  eq.m * raw + eq.b;
    *value = mapf( cali, 4.0, 20.0, self->range.min, self->range.max );
    Serial.printf("%s: %f -> %f -> %f %s\n", self->id, raw, cali, *value, self->unit );
    return 0;
}


void init_sht3x( void ) {
    if (sht.init()) {
        Serial.print("init(): success\n");
    } else {
        Serial.print("init(): failed\n");
    }
    sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
}



struct source2sensor{ 
    char const* source; 
    void (*init)( void );
    struct sensors const sensors[2];
    int len;
} 
const src2sens[] = {
    { "Temperature", init_sht3x,
        {{ "temp_air", get_sht3x_temperature, "ºC" },
         { "hum_air", get_sht3x_humidity, "%" }}
        , 2 },
    { "CH4", board_initadc, {{ "ch4", get_gas_sensor, "ppm", { 0.0, 10000.0} }}, 1 }, /*ppm = LIE*10000*/
    { "H2S", board_initadc, {{ "h2s", get_gas_sensor, "ppm", { 0.0, 100.0} }}, 1 },
    { "NH3", board_initadc, {{ "nh3", get_gas_sensor, "ppm", { 0.0, 100.0} }}, 1 }
};


/*Print local time and date, used for debugging purposes*/
static void printLocalTime(){
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time");
        return;
    }

    if( 1 < verbose ) {
        Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
        Serial.print("Day of week: ");
        Serial.println(&timeinfo, "%A");
        Serial.print("Month: ");
        Serial.println(&timeinfo, "%B");
        Serial.print("Day of Month: ");
        Serial.println(&timeinfo, "%d");
        Serial.print("Year: ");
        Serial.println(&timeinfo, "%Y");
        Serial.print("Hour: ");
        Serial.println(&timeinfo, "%H");
        Serial.print("Hour (12 hour format): ");
        Serial.println(&timeinfo, "%I");
        Serial.print("Minute: ");
        Serial.println(&timeinfo, "%M");
        Serial.print("Second: ");
        Serial.println(&timeinfo, "%S");

        Serial.println("Time variables");
        char timeHour[3];
        strftime(timeHour,3, "%H", &timeinfo);
        Serial.println(timeHour);
        char timeWeekDay[10];
        strftime(timeWeekDay,10, "%A", &timeinfo);
        Serial.println(timeWeekDay);
        Serial.println();
    }
}

/*Print access point configuration*/
static void print_APcfg( struct ap_config const* ap ) {
    String myIP = WiFi.softAPIP().toString();
    Serial.printf("Set up access point. SSID: %s, PASS: %s\n", ap->ssid, ap->pass);
    Serial.printf("#### SERVER STARTED ON THIS: %s ###\n", myIP.c_str());
    printIp( "Access point ADDRESS: ", &ap->addr );
}


/*Calculate the equation of a straight line used to calibrate*/
static void getCalibrationEquation( struct caleq* eq, double x0, double y0, double x1, double y1 ) {
    double num = 0 == (x1 - x0 ) ? 1 : (x1 - x0 );
    eq->m = (y1 - y0) / num;
    eq->b = y0 - eq->m * x0;
    if( verbose ) {
        Serial.printf("Calibration equation: y = %.2f * x %c%.2f\n", eq->m, eq->b > 0.0 ? '+' : ' ', eq->b);
    }
}

void sensors_init( char const* name ) {
    for( int i = 0; i < sizeof(src2sens)/sizeof(src2sens[0]); ++i ) {
        if ( strcmp( name, src2sens[i].source) == 0 ) {
            src2sens[i].init();
        }
    }
}

static char* json_measurement( char* dest, struct sensors const* measurement, size_t* remlen ) {                       

    double value = 0;
    int err = measurement->getsample( measurement, &value );

    dest = json_objOpen( dest, measurement->id, remlen ); 
    dest = json_double( dest, "value", value, remlen ); 
    /* Add a context object property in a JSON string. */
    dest = json_objOpen( dest, "context", remlen );
    dest = json_str( dest, "unit",   measurement->unit , remlen  );
    dest = json_str( dest, "status", err ? "fail":"ok" , remlen );
    dest = json_objClose( dest, remlen );
    
    dest = json_objClose( dest, remlen ); 
    
    return dest;
}

static char* json_sensor( char* dest, char const* name, size_t* remlen ) {
    for( int i = 0; i < sizeof(src2sens)/sizeof(src2sens[0]); ++i ) {
        if ( strcmp( name, src2sens[i].source) == 0 ) {
            for( int j = 0; j < src2sens[i].len; ++j ) { 
                dest = json_measurement( dest, &src2sens[i].sensors[j], remlen );
            }
        }
    }
    return dest;
}

static char* json_timestampMs( char* dest, char const* name, size_t* remlen ) {
    time_t now;
    time( &now );
    dest = json_verylong( dest, name, (uint64_t)now*1000, remlen );
    return dest;
}

static char* json_location( char* dest, char const* name, size_t* remlen ) {
    dest = json_objOpen( dest, name, remlen ); 
    dest = json_double( dest, "lat", cfg.service.geo.lat, remlen );
    dest = json_double( dest, "lng", cfg.service.geo.lng, remlen );
    dest = json_objClose( dest, remlen ); 
    return dest;
}

enum jsontype {
    NONE = 0,
    JSON_MEASUREMENT,
    JSON_STATUS,
    JSON_INFO
};

static void json_frame( char* dest, enum jsontype jsontype ) {
    size_t remlen = JSON_TX_SIZE;
    dest = json_objOpen( dest, NULL, &remlen ); 
    dest = json_timestampMs( dest, "timestamp", &remlen );
    
    switch ( jsontype ) {
        case JSON_MEASUREMENT:
            dest = json_sensor( dest, cfg.cal.id_sens_1, &remlen );
            dest = json_sensor( dest, cfg.cal.id_sens_2, &remlen );
        break;
        case JSON_STATUS:
            dest = json_double( dest, "batt", 3.7, &remlen ); 
        break;
        case JSON_INFO:
            dest = json_location( dest, "location", &remlen );
        break;
        default:
            Serial.print("error, undefined json type\n");
        break;
    }
    
    dest = json_objClose( dest, &remlen ); 
    json_end( dest, &remlen );
}

/*Get the publishing period of the topic from the topic configuration structure. 
The publishing period is returned milliseconds. Return -1 is configuration is not correct*/
static int getupdatePeriod( struct pub_topic const* tp ) {
    struct { char const* unit; int factor; } const units[] = {
        { "Second", 1 * 1000 },
        { "Minute", 60 * 1000 },
        { "Hour", 60 * 60 * 1000}
    };

    for( int i = 0; i < sizeof(units)/sizeof(units[0]); ++i ) {
        if ( strcmp( tp->unit, units[i].unit ) == 0 ) {
            return tp->period * units[i].factor;
        }
    }
    return -1;
}

/*Callback function used to pusblish measurement on the mqtt topic*/
static void pubMeasurement_callback( TimerHandle_t xTimer ) {
    xEventGroupSetBits( events, PUB_MEASURES );        
}

/*Callback function used to pusblish status on the mqtt topic*/
static void pubStatus_callback( TimerHandle_t xTimer ) {
    xEventGroupSetBits( events, PUB_STATUS );        
}

/*Callback function used to pusblish info on the mqtt topic*/
static void pubInfo_callback( TimerHandle_t xTimer ) {
    xEventGroupSetBits( events, PUB_INFO );        
}

/*Test the status of the Wifi connection and attempt reconnection if it fails.*/
static bool testWifi( void ) {
    int tries = 0;
    Serial.println("Waiting for Wifi to connect");
    while ( tries < 30 ) {
        interface_setMode( ON );
        wl_status_t const status = WiFi.status();
        vTaskDelay(pdMS_TO_TICKS(250));
        interface_setMode( OFF );
        if ( status == WL_CONNECTED ) {
            return true;
        }
        bool const isupdate = webserver_isNetworkUpdated( );
        if( isupdate ) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        Serial.print( status );
        Serial.print(".");
        ++tries;
    }
    return false;
}


/*Enter in the configuration mode: enable wifi access point and webserver.*/
void ctrl_enterConfigMode( void ) {
    if( isServerActive )
        return;
    WiFi.mode(WIFI_AP_STA);
    vTaskDelay(pdMS_TO_TICKS(1000));
    isServerActive = true;
    webserver_start( );
}

/*Exit from the configuration mode: disable wifi access point and webserver.*/
void ctrl_exitConfigMode( void ) {
    if( !isServerActive ) 
        return;
    webserver_stop( );
    WiFi.mode(WIFI_STA);
    isServerActive = false;
}

bool ctrl_isConfigModeEnable( void ) {
    return isServerActive;
}

/*Freertos task*/
void ctrl_task( void * parameter ) {

    interface_setMode( OFF );
    tmPubMeasurement = xTimerCreate( "tmMeasurement", pdMS_TO_TICKS( 20000 ), pdTRUE, NULL, pubMeasurement_callback );
    tmPubStatus = xTimerCreate( "tmStatu", pdMS_TO_TICKS( 15000 ), pdTRUE, NULL, pubStatus_callback );
    tmPubInfo = xTimerCreate( "tmInfo", pdMS_TO_TICKS( 10000 ), pdFALSE, NULL, pubInfo_callback );
    int failCounter = 0;
    struct acq_cal const* cal = &cfg.cal;
    getCalibrationEquation( &eq, cal->val[0].x, cal->val[0].y, cal->val[1].x, cal->val[1].y );
    /* Attempt to create the event group. */
    events = xEventGroupCreate();
    EventBits_t bitfied = xEventGroupSetBits( events, START_AP_WIFI | CONNECT_WIFI );

    sensors_init( cfg.cal.id_sens_1 );
    sensors_init( cfg.cal.id_sens_2 );

    for(;;){ 
        
        bool const iscfgmode = ctrl_isConfigModeEnable();
        if ( 10 < failCounter && !iscfgmode ) {
            Serial.println("Restarting ESP...");  
            ESP.restart();  
        }

        bitfied = xEventGroupGetBits( events );
        /*Configura WiFi Access Point*/
        if( bitfied & START_AP_WIFI ) {
            Serial.println("Starting AP");
            interface_setMode( OFF );
            struct ap_config const* ap  = &cfg.ap;
            IPAddress ip(ap->addr.ip[0], ap->addr.ip[1], ap->addr.ip[2], ap->addr.ip[3]);
            IPAddress nmask(255, 255, 255, 0);
            WiFi.softAP( ap->ssid, ap->pass);
            WiFi.softAPConfig(ip, ip, nmask );

            if( verbose )
                print_APcfg( ap );

            WiFi.mode(WIFI_STA);
            xEventGroupClearBits( events, START_AP_WIFI );
        }

        /*Connect to WiFi*/
        bool const updatenet = webserver_isNetworkUpdated( );
        if( (bitfied & CONNECT_WIFI) || updatenet) {
            xEventGroupClearBits( events, CONNECT_MQTT );
            interface_setMode( OFF );
            struct wifi_config* wf = &cfg.wifi;
            
            if( (wf->ssid[0] == 0 ) || wf->mode[0] == 0 ) {
                Serial.println("No wifi config found");
                vTaskDelay( pdMS_TO_TICKS(10000) );
                xEventGroupSetBits( events, CONNECT_WIFI );
                continue;
            }
            
            if ( strcmp( wf->mode, "static") == 0 ) {
                IPAddress addr( wf->ip.ip[0], wf->ip.ip[1], wf->ip.ip[2], wf->ip.ip[3] );
                IPAddress gateway( wf->gateway.ip[0], wf->gateway.ip[1], wf->gateway.ip[2], wf->gateway.ip[3] );
                IPAddress subnet( wf->netmask.ip[0], wf->netmask.ip[1], wf->netmask.ip[2], wf->netmask.ip[3] );
                IPAddress primaryDNS( wf->primaryDNS.ip[0], wf->primaryDNS.ip[1], wf->primaryDNS.ip[2], wf->primaryDNS.ip[3] ); //optional
                IPAddress secondaryDNS( wf->secondaryDNS.ip[0], wf->secondaryDNS.ip[1], wf->secondaryDNS.ip[2], wf->secondaryDNS.ip[3] ); //optional
                if (!WiFi.config( addr, gateway, subnet, primaryDNS, secondaryDNS)) {
                    Serial.println("STA Failed to configure");
                }
            }
            
            if( verbose ) {
                print_NetworkCfg( wf );
                Serial.printf("Mac: %s\n", cfg.service.client_id );
            }

            WiFi.begin( wf->ssid, wf->pass );
            if ( !testWifi() ) {
                Serial.println("\nWifi connection failed");
                vTaskDelay( pdMS_TO_TICKS(5000) );
                continue;
            }
            
            Serial.printf("Connected to %s, IP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str() );            
            interface_setMode( BLINK );
            xEventGroupSetBits( events,   CONNECT_MQTT );
            xEventGroupClearBits( events, CONNECT_WIFI );
            failCounter = 0;
        }
        
        /*Check Wifi status*/
        if( ( WiFi.status() != WL_CONNECTED ) ) {
            if( !testWifi() ) {
                Serial.println("\nWifi connection failed, try again in 5 seconds\n");
                client.disconnect();
                xEventGroupSetBits( events, CONNECT_WIFI );
                interface_setMode( OFF );
                ++failCounter;
                xTimerStop( tmPubInfo, 100 );
                xTimerStop( tmPubMeasurement, 100 );
                xTimerStop( tmPubStatus, 100 );
                vTaskDelay( pdMS_TO_TICKS(5000) );
                continue;
            }
        }

        /*Update calibration parameters*/
        bool const updatecal = webserver_isCalibrationUpdated( );
        if( updatecal ) {
            struct acq_cal const* cal = &cfg.cal;
            getCalibrationEquation( &eq, cal->val[0].x, cal->val[0].y, cal->val[1].x, cal->val[1].y );
        }
        
        /*Send data to UDP*/
        dataframe data;
        bool const newframe = waitnewData( &data);
        if( newframe ) {
            if( 0 != cfg.udp.port ) {
                IPAddress ip( cfg.udp.ip.ip[0], cfg.udp.ip.ip[1], cfg.udp.ip.ip[2], cfg.udp.ip.ip[3] );
                udp.connect( ip, cfg.udp.port );
                udp.print( dataStructureToCsv( &data ) );
                udp.close();
            }
        } 

#if 0
        /*Connect to the MQTT broker and NTP server */
        bool const updateserv = webserver_isServiceUpdated( );
        if( ((bitfied & CONNECT_MQTT) || updateserv) && !iscfgmode ) {
            
            memcpy( &scfg, &cfg.service, sizeof(cfg.service) );
            if( scfg.host_ip[0] == 0 || scfg.client_id == 0 ) {
                Serial.println("No MQTT config found");
                vTaskDelay( pdMS_TO_TICKS(10000) );
                xEventGroupSetBits( events, CONNECT_MQTT );
                continue;
            }
            
            Serial.println("Attempting MQTT connection...");
            client.setServer( scfg.host_ip, scfg.port);
            bool connected = client.connect( scfg.client_id, scfg.username, scfg.password );
            if ( !connected ) {
                Serial.printf("Failed broker connection, rc= %d %s\n", client.state(), "try again in 5 seconds");
                ++failCounter;
                vTaskDelay( pdMS_TO_TICKS(5000) ); 
                continue;
            }
                           
            xTimerStart( tmPubInfo, 100 );
            xTimerStart( tmPubMeasurement, 100 );
            xTimerStart( tmPubStatus, 100 );
            
            const long  gmtOffset_sec = 3600;
            const int   daylightOffset_sec = 3600;
            configTime( gmtOffset_sec, daylightOffset_sec, cfg.ntp.host );
            Serial.println("Connected to broker");
            interface_setMode( ON );
            xEventGroupClearBits( events, CONNECT_MQTT );
            failCounter = 0;
        }
        
        /*Check MQTT conection status*/
        bool mqttconneted = client.loop();
        if( !mqttconneted ) {
            Serial.println("\nMQTT connection failed, try again in 5 seconds\n");
            client.disconnect();
            xEventGroupSetBits( events, CONNECT_MQTT );
            interface_setMode( BLINK );
            ++failCounter;
            xTimerStop( tmPubInfo, 100 );
            xTimerStop( tmPubMeasurement, 100 );
            xTimerStop( tmPubStatus, 100 );
            vTaskDelay( pdMS_TO_TICKS(5000) );
            continue;
        }

        if( bitfied & PUB_INFO ) {
            xEventGroupClearBits( events, PUB_INFO );
            json_frame( payload_json, JSON_INFO );
            client.publish( scfg.info.topic, payload_json);
            Serial.printf("Publishing info %s\n", payload_json);
        }

        if( bitfied & PUB_STATUS ) {
            xEventGroupClearBits( events, PUB_STATUS );
            json_frame( payload_json, JSON_STATUS );
            client.publish( scfg.status.topic, payload_json );
            Serial.printf("Publishing status %s\n", payload_json);
            xTimerChangePeriod( tmPubStatus, pdMS_TO_TICKS( getupdatePeriod( &scfg.status )), 100 );
            if( verbose ) {
                printLocalTime();
            }           
        }

        if( bitfied & PUB_MEASURES ) {
            xEventGroupClearBits( events, PUB_MEASURES );
            json_frame( payload_json, JSON_MEASUREMENT );
            client.publish( scfg.measures.topic, payload_json);
            Serial.printf("Publishing measurements %s\n", payload_json);
            xTimerChangePeriod( tmPubMeasurement, pdMS_TO_TICKS( getupdatePeriod( &scfg.measures )), 100 ); 
        }
#endif
        vTaskDelay( pdMS_TO_TICKS(20) );
    }
}




