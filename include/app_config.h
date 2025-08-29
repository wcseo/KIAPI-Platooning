/*
 * Project : KIAPI-Platooning
 * Author: WooChang Seo (wcseo@kiapi.or.kr) 
 * Date: 2025-08-29
 */ 
 
#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#include <string> 
#include <stdint.h>

struct app_config_t{ 
    std::string dgd_ip = "127.0.0.1";
    std::string obu_ip = "127.0.0.1";
    uint32_t obu_port = 47347;  
    uint32_t dev_id = 0; 
    uint32_t tx_interval = 100; // msec;
    std::string proxy_ip = "";  
    bool verbose = false;

    std::string vc_id = "KIAPI-VEH";
    std::string vc_number = "000A0000";
};
 
#endif