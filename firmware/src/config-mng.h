/*
    Project: <https://github.com/AngelJMC/4-20ma-wifi-bridge>   
    Copyright (c) 2022 Angel Maldonado <angelgesus@gmail.com>. 
    Licensed under the MIT License: <http://opensource.org/licenses/MIT>.
    SPDX-License-Identifier: MIT 
*/

#ifndef _CONFIG_MNG_H_
#define _CONFIG_MNG_H_                   

#include <stdint.h>

enum {
    SSID_SIZE = 32,
    PASS_SIZE = 32,
    APADDR_SIZE = 32
};

/** ip struct */
struct ip {
    uint8_t ip[4];
};

enum time_unit{
    TIME_UNIT_SECOND,
    TIME_UNIT_MINUTE,
    TIME_UNIT_HOUR
};

struct pub_topic {
    char topic[64];
    int32_t period; /* in seconds */
    char unit[16];
};



struct wifi_config {
    char ssid[32];
    char pass[32];
    char mode[16];
    struct ip ip;
    struct ip netmask;
    struct ip gateway;
    struct ip primaryDNS;
    struct ip secondaryDNS;
};

struct ntp_config {
    char host[64];
    int  port;
    int  period;
};


struct udp_config {
    struct ip ip;
    int port;
};

struct service_config {
    char host_ip[64];
    uint16_t port;
    char client_id[32];
    char username[64];
    char password[32];
    struct pub_topic measures;
    struct pub_topic status;
    struct pub_topic info;
    struct location{
        double lat;
        double lng;
    } geo;
};

struct ap_config {
    char ssid[32];
    char pass[32];
    struct ip addr;
    char web_user[16];
    char web_pass[16];
};

struct acq_cal {
    struct point {
        double  y;
        int16_t x;
    } val[2];

    char id_sens_1[16];
    char id_sens_2[16];
};

/** Create a structure to hold the configuration data. */
struct config  {
    struct ap_config ap;
    struct wifi_config wifi;
    struct service_config  service;
    struct ntp_config ntp;
    struct udp_config udp;
    struct acq_cal cal;
};

extern struct config cfg;


void config_load( void ); 

void config_savecfg( );

void config_overwritedefaultcal( struct acq_cal const* calibration );

void config_setdefault( void );


void print_ntpCfg( struct ntp_config const* ntp );

void print_udpCfg( struct udp_config const* udp );


void print_NetworkCfg( struct wifi_config const* ntwk );

void printIp( char const* name, struct ip const* src );

void print_apCfg( struct ap_config const* ap );

void print_ServiceCfg( struct service_config const* srvc );

void print_Calibration( struct acq_cal const* cal );

#endif 
 
