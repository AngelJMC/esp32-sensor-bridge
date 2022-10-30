/*
    Project: <https://github.com/AngelJMC/4-20ma-wifi-bridge>   
    Copyright (c) 2022 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#include "config-mng.h"
#include <Arduino.h>
#include <EEPROM.h>


#define CFG_VER 1

#define EEPROM_SIZE 1024

struct config cfg;
struct acq_cal cal;
static int const cfgaddr = 0; 
static int const caladdr = 900;


void static strgetname( char* name, char const* prefix ) {
    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(name, "%s_%02X%02X", prefix, baseMac[4], baseMac[5]);
}

void static strgetclientid( char* name ) {
    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(name, "%02x%02x%02x%02x%02x%02x", 
        baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5] );   
}

static void setdefault( struct config* cfg ) { 
    memset( cfg, 0 , sizeof( struct config));
    
    strgetname( cfg->ap.ssid, "Logger_4-20mA" );
    strcpy( cfg->ap.pass, "3Qd400Ak&1i8" );
    
    struct ip addr = { 192,168,4,1 };
    memcpy( &cfg->ap.addr, &addr, sizeof( addr));

    strcpy( cfg->ap.web_user, "admin" );
    strcpy( cfg->ap.web_pass, "Y32Pv9RY" );

    strcpy( cfg->wifi.mode, "dhcp" );
    strcpy( cfg->ntp.host, "pool.ntp.org");

    strgetclientid( cfg->service.client_id );
    strcpy( cfg->service.host_ip, "industrial.api.ubidots.com");
    cfg->service.port = 1883;
    
    sprintf(cfg->service.measures.topic, "%s/%s", "/v2.0/devices", cfg->service.client_id ) ;
    cfg->service.measures.period = 20;
    strcpy( cfg->service.measures.unit, "Second" );
    
    sprintf(cfg->service.status.topic, "%s/%s", "/v2.0/devices", cfg->service.client_id ) ;
    cfg->service.status.period = 1;
    strcpy( cfg->service.status.unit, "Minute" );

    sprintf(cfg->service.info.topic, "%s/%s", "/v2.0/devices", cfg->service.client_id ) ;
    cfg->service.info.period = 0;
    strcpy( cfg->service.info.unit, "Second" );

    cfg->cal = cal;
}

void config_setdefault( void ) { 
    setdefault( &cfg );
    EEPROM.put( cfgaddr, cfg );
    EEPROM.commit();
}

void config_savecfg( void ) {
    Serial.println("Saving config to EEPROM");
    EEPROM.put( cfgaddr, cfg );
    EEPROM.commit(); 
}

void config_overwritedefaultcal( struct acq_cal const* calibration ) {
    Serial.println("Overwrite default calibration to EEPROM");
    cal = *calibration;
    EEPROM.put( caladdr, cal );
    EEPROM.commit(); 
}

void config_load( void ) {
    
    if ( !EEPROM.begin(EEPROM_SIZE) ) {
        Serial.println("failed to initialise EEPROM"); 
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    
    int cfgversion=0;
    int eeAdress = cfgaddr;
    EEPROM.get( eeAdress, cfg );
    eeAdress += sizeof( struct config );
    EEPROM.get( eeAdress, cfgversion );
    eeAdress += sizeof( int );

    if ( eeAdress >= caladdr ) {
        Serial.println("ERROR > EEPROM insufficient size for config");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    EEPROM.get( caladdr, cal );
    eeAdress += sizeof( struct acq_cal );
    
    if ( eeAdress > EEPROM_SIZE ) {
        Serial.println("ERROR > EEPROM insufficient size for calibration");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    if ( cfgversion != CFG_VER ) {
        setdefault( &cfg );
        eeAdress = cfgaddr;
        EEPROM.put( eeAdress, cfg );
        eeAdress += sizeof( struct config );
        cfgversion = CFG_VER;
        EEPROM.put( eeAdress, cfgversion );
        EEPROM.put( caladdr, cal );
        eeAdress += sizeof( struct acq_cal );
        Serial.printf("LOAD DEFAULT\n");
    }

    EEPROM.commit(); 
} 



void printIp( char const* name, struct ip const* src ) {
    Serial.printf("%s%d.%d.%d.%d\n", name, src->ip[0], src->ip[1], src->ip[2], src->ip[3]);
}

void print_ntpCfg( struct ntp_config const* ntp ) {
    Serial.printf("NTP HOST: %s\n", ntp->host);
    Serial.printf("NTP PORT: %d\n", ntp->port);
    Serial.printf("NTP PERIOD: %d\n", ntp->period);
}

void print_NetworkCfg( struct wifi_config const* ntwk ) {
        Serial.printf("WIFI SSID: %s\n", ntwk->ssid);
        Serial.printf("WIFI PASS: %s\n", ntwk->pass);
        Serial.printf("WIFI MODE: %s\n", ntwk->mode );
        
        if ( strcmp( ntwk->mode, "static") == 0 ) {
            printIp("WIFI IP: ", &ntwk->ip);
            printIp("WIFI NETMASK: ", &ntwk->netmask);
            printIp("WIFI GATEWAY: ", &ntwk->gateway);
            printIp("WIFI PRIMARY DNS: ", &ntwk->primaryDNS);
            printIp("WIFI SECONDARY DNS: ", &ntwk->secondaryDNS);
        }
}

void print_ServiceCfg( struct service_config const* srvc ) {

        Serial.printf("MQTT HOST IP: %s\n", srvc->host_ip);
        Serial.printf("MQTT PORT: %d\n", srvc->port);
        Serial.printf("MQTT CLIENT_ID: %s\n", srvc->client_id);
        Serial.printf("MQTT USER: %s\n", srvc->username);
        Serial.printf("MQTT PASS: %s\n", srvc->password);
        Serial.printf("MQTT TEMP_TOPIC: %s\n", srvc->measures.topic);
        Serial.printf("MQTT TEMP_INTER: %d\n", srvc->measures.period);
        Serial.printf("MQTT TEMP_UND: %s\n", srvc->measures.unit);
        Serial.printf("MQTT PING_TOPIC: %s\n", srvc->status.topic);
        Serial.printf("MQTT PING_INTER: %d\n", srvc->status.period);
        Serial.printf("MQTT PING_UND: %s\n", srvc->status.unit);
        Serial.printf("LOC LAT: %f\n", srvc->geo.lat );
        Serial.printf("LOC LON: %f\n", srvc->geo.lng );
}

void print_apCfg( struct ap_config const* ap ) {
    Serial.printf("AP SSID: %s\n", ap->ssid );
    Serial.printf("AP PASS: %s\n", ap->pass );
    printIp("AP IP: ", &ap->addr);
}

void print_Calibration( struct acq_cal const* cal ) {
    Serial.printf("CAL [x0,y0]: [%d,%f]\n", cal->val[0].x, cal->val[0].y  );
    Serial.printf("CAL [x1,y1]: [%d,%f]\n", cal->val[1].x, cal->val[1].y  );
    Serial.printf("SEN_1: %s\n", cal->id_sens_1 );
    Serial.printf("SEN_2: %s\n", cal->id_sens_2 );
}