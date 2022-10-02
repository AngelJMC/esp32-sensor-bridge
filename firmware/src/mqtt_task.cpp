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

enum {
    verbose = 2,
    apAutoStart = 0
};

/*Calibration equation*/
struct caleq {
    double m;
    double b;
};

WiFiClient wifiClient;
PubSubClient client(wifiClient);

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

char jsonstatus[128];

enum flags {
    START_AP_WIFI = 1 << 0,
    CONNECT_WIFI  = 1 << 1,
    CONNECT_MQTT  = 1 << 2
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

/*Apply the calibration funtion to an input value.*/
static double applyCalibration( struct caleq* eq, double x ) {
    return eq->m * (x-5) + eq->b ;
} 

/*Format json string with status info.*/
static void status2json( char* json ) {
    
    double batt = 3.7;
    time_t now;
    time(&now);
    sprintf(json,
        "{ \"timestamp\": %ld, \"batt\": %f }",
        now,  
        batt
    );

}

static void measurements2json( char* json ) {
    int16_t raw = getadcValue( );
    double converted = applyCalibration( &eq, raw );
    float sensor_value = 1001.1;
    char const sensor_name[] = "ch4";
    char const unit[] = "ppb";
    char const status[] = "ok";
    time_t now;
    time(&now);
    sprintf(json,
        "{ \"%s\" : { \"value\": %f, \"context\":{ \"unit\": \"%s\", \"status\": \"%s\"}}, \"timestamp\": %" PRIu64 "}",
        sensor_name,
        sensor_value,
        unit,
        status,
        (uint64_t)now*1000
    ); 
}


static void info2json( char* json ) {
    //TODO Add GPS 
    time_t now;
    time(&now);
    sprintf(json,
        "{ \"location\" : { \"lat\": %f, \"lng\": %f }, \"timestamp\": %ld }",
        cfg.service.geo.lat,
        cfg.service.geo.lng,
        now
    ); 
}

/*Callback function used to pusblish measurement on the mqtt topic*/
static void pubMeasurement_callback( TimerHandle_t xTimer ) {
    struct service_config const* sc = &scfg;
    measurements2json( jsonstatus );
    client.publish( sc->temp.topic, jsonstatus);
    Serial.printf("Publishing measurements %s\n", jsonstatus);          
}

/*Callback function used to pusblish status on the mqtt topic*/
static void pubStatus_callback( TimerHandle_t xTimer ) {
    struct service_config const* sc = &scfg;
    status2json( jsonstatus );
    client.publish( sc->ping.topic, jsonstatus );
    Serial.printf("Publishing status %s\n", jsonstatus);
    
    if( verbose ) {
        printLocalTime();
    }
}

/*Callback function used to pusblish info on the mqtt topic*/
static void pubInfo_callback( TimerHandle_t xTimer ) {
    struct service_config const* sc = &scfg;
    info2json( jsonstatus );
    client.publish( sc->temp.topic, jsonstatus);
    Serial.printf("Publishing info %s\n", jsonstatus);          
}

/*Funtion to convert byte* to char* */
static void byteToChar(char* dest, byte* src, int len) {
    for (int i = 0; i < len; i++) {
        dest[i] = src[i];
    }
    dest[len] = '\0';
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
    tmPubMeasurement = xTimerCreate( "tmMeasurement", pdMS_TO_TICKS( 10000 ), pdTRUE, NULL, pubMeasurement_callback );
    tmPubStatus = xTimerCreate( "tmStatu", pdMS_TO_TICKS( 10000 ), pdTRUE, NULL, pubStatus_callback );
    tmPubInfo = xTimerCreate( "tmInfo", pdMS_TO_TICKS( 10000 ), pdTRUE, NULL, pubInfo_callback );

    struct acq_cal const* cal = &cfg.cal;
    getCalibrationEquation( &eq, cal->val[0].x, cal->val[0].y, cal->val[1].x, cal->val[1].y );
    /* Attempt to create the event group. */
    events = xEventGroupCreate();
    EventBits_t bitfied = xEventGroupSetBits( events, START_AP_WIFI | CONNECT_WIFI );


    for(;;){ 
        
        bitfied = xEventGroupGetBits( events );
        
        /*Configura WiFi Access Point*/
        if( bitfied & START_AP_WIFI ) {
            Serial.println("Starting AP");
            interface_setMode( OFF );
            struct ap_config const* ap  = &cfg.ap;
            IPAddress ip(ap->addr.ip[0], ap->addr.ip[1], ap->addr.ip[2], ap->addr.ip[3]);
            IPAddress nmask(255, 255, 0, 0);
            WiFi.softAP( ap->ssid, ap->pass);
            WiFi.softAPConfig(ip, ip, nmask );

            if( verbose )
                print_APcfg( ap );

            if( apAutoStart )
                ctrl_enterConfigMode( );
            else
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
                continue;
            }
            
            Serial.printf("Connected to %s, IP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str() );            
            interface_setMode( BLINK );
            xEventGroupSetBits( events,   CONNECT_MQTT );
            xEventGroupClearBits( events, CONNECT_WIFI );
        }

        /*Update calibration parameters*/
        bool const updatecal = webserver_isCalibrationUpdated( );
        if( updatecal ) {
            struct acq_cal const* cal = &cfg.cal;
            getCalibrationEquation( &eq, cal->val[0].x, cal->val[0].y, cal->val[1].x, cal->val[1].y );
        }

        /*Connect to the MQTT broker and NTP server */
        bool const updateserv = webserver_isServiceUpdated( );
        if( (bitfied & CONNECT_MQTT) || updateserv ) {
            
            memcpy( &scfg, &cfg.service, sizeof(cfg.service) );
            if( scfg.host_ip[0] == 0 || scfg.client_id == 0 ) {
                Serial.println("No mqtt config found");
                vTaskDelay( pdMS_TO_TICKS(10000) );
                xEventGroupSetBits( events, CONNECT_MQTT );
                continue;
            }
            
            client.setServer( scfg.host_ip, scfg.port);
            
            Serial.println("Attempting MQTT connection...");
            if ( client.connect( scfg.client_id, scfg.username, scfg.password ) ) {                
                
                xTimerChangePeriod( tmPubMeasurement, pdMS_TO_TICKS( 10000), 100 );
                xTimerStart( tmPubMeasurement, 100 );
                //xTimerChangePeriod( tmPubStatus, pdMS_TO_TICKS( getupdatePeriod( &scfg.ping )), 100 );
                xTimerChangePeriod( tmPubStatus, pdMS_TO_TICKS( 20000), 100 );
                xTimerStart( tmPubStatus, 100 );
                xTimerChangePeriod( tmPubInfo, pdMS_TO_TICKS( 30000 ), 100 );
                xTimerStart( tmPubStatus, 100 );
                
                if (verbose ) {
                    Serial.println("Connected to broker");
                }
            } 
            else {
                Serial.printf("Failed broker connection, rc= %d %s\n", client.state(), "try again in 5 seconds");
                vTaskDelay( pdMS_TO_TICKS(5000) ); 
                continue;
            }
            
            if( !updateserv ) {
                ctrl_exitConfigMode(  );
            }

            const long  gmtOffset_sec = 3600;
            const int   daylightOffset_sec = 3600;
            configTime( gmtOffset_sec, daylightOffset_sec, cfg.ntp.host );
            interface_setMode( ON );
            xEventGroupClearBits( events, CONNECT_MQTT );
        }
        
        if( ( WiFi.status() != WL_CONNECTED ) ) {
            if ( !testWifi() ) {
                Serial.println("\nWifi connection failed");
                xEventGroupSetBits( events, CONNECT_WIFI );
                if( apAutoStart )
                    ctrl_enterConfigMode( );
                interface_setMode( OFF );
                continue;
            }
        }

        if( !client.connected() ) {
            Serial.println("\nMQTT connection failed\n");
            xEventGroupSetBits( events, CONNECT_MQTT );
            interface_setMode( BLINK );
        }
        
        client.loop();
        vTaskDelay( pdMS_TO_TICKS(250) );
    }
}




