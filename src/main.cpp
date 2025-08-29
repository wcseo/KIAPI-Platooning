/*
 * Project : KIAPI-Platooning
 * Author: WooChang Seo (wcseo@kiapi.or.kr) 
 * Date: 2025-08-29
 */  
  
#include "app_config.h"
#include <cpp-framework/common/option.h>
#include <cpp-framework/common/log.h> 
#include <cpp-framework/common/time.h> 
#include "obu_handler.h" 
app_config_t config;
 
 
/// @brief Option 파라미터 Parser 
/// @param argc 프로그램 실행 Arguement Count
/// @param argv 프로그램 실행 Arguement Value
void parse_option(int argc, char** argv){

    auto vals = option_parser(argc,argv,"n:N:d:i:I:p:D:P:vh");
  
    for(int i = 0 ; i < vals.size() ; i++){

        switch(vals[i].opt){
        case 'N': // VEHICLE ID (NAME)
            config.vc_id = vals[i].arg;
            break;
        case 'n': // Vehicle Number
            config.vc_number = vals[i].arg;
            break;
        case 'd': // Device Number
            config.dev_id = std::atoi(vals[i].arg.c_str());
            break;
        case 'i': // OBU IP
            config.obu_ip = vals[i].arg;
            break;
        case 'I': // Tx Interval
            config.tx_interval =  std::atoi(vals[i].arg.c_str());
            break;
        case 'p': // OBU Port
            config.obu_port = std::atoi(vals[i].arg.c_str());
            break;
        case 'D': // DGD IP 
            config.dgd_ip = vals[i].arg;
            break;
        case 'P': // SERVER IP 
            config.proxy_ip = vals[i].arg;
            break;  
        case 'v': // SET Verbose 
            config.verbose = true;
            break;
        case 'h':
        default: 
            printf("\t %s usage [option argument]]\n", argv[0]);
            printf("\t -N [Vehicle ID]             , Set Vehicle ID [default : KIAPI-VEH] \n");
            printf("\t -n [Vehicle Number]         , Set Vehicle Number [default : 000A0000] \n");
            printf("\t -d [Device ID Number]       , Set Transfer Device ID Number (4byte Number)\n");
            printf("\t -i [OBU IP]                 , Set OBU Device IP Address \n");
            printf("\t -I [Interval msec]          , Set Platooning Service Message Transmit Interval \n");
            printf("\t -p [OBU Port]               , Set OBU Device TCP Sock PORT [default : 47347]\n");
            printf("\t -D [DGD IP]                 , Set DGD Device IP Address \n");
            printf("\t -P [SERVER IP]              , Set Proxy Mode, Access Server IP \n");  
            printf("\t -v                          , Set Verbose mode \n");
            exit(0);
        }
    }
}
 


/// @brief 프로그램 Main 실행부
/// @param argc 프로그램 실행 Arguement Count
/// @param argv 프로그램 실행 Arguement Value
/// @return 
int main(int argc, char **argv)
{ 
    parse_option(argc, argv); 
  
    obu_handler obu(config); 

    while (true)
    { 
        sleep_for(100);
    } 

    return 0;
}