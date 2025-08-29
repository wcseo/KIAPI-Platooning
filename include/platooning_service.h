/*
 * Project : KIAPI-Platooning
 * Author: WooChang Seo (wcseo@kiapi.or.kr) 
 * Date: 2025-08-29
 */ 

#ifndef _OBU_KETI_PLATOONING_PLATOONING_H_
#define _OBU_KETI_PLATOONING_PLATOONING_H_
   
#include <nr-v2x/nr_v2x_mng.h>
#include <cpp-framework/gps/gps_handler.h>
#include <cpp-framework/thread/local_lock.h> 
#include <nr-v2x/keti/db_v2x_platooning.h> 
#include <nr-v2x/keti/db_v2x_status.h>
#include <nr-v2x/keti/db_v2x.h>

typedef enum{ 
    PLATOONING_ROLL_TYPE_LV = 0,
    PLATOONING_ROLL_TYPE_FV = 1 
}PLATOONING_ROLL_TYPE_T; 

class platooning_service
{
public:
    platooning_service(uint32_t psid =nr_v2x_psid_list_t::eV2XMSG_TYPE_5G_EM_V2V , uint8_t type = PLATOONING_ROLL_TYPE_T::PLATOONING_ROLL_TYPE_FV);
    ~platooning_service();       

    void set_debug(bool set);
    void set_device_id(uint32_t id);
    void set_vehicle_id(std::string id , std::string number);
    void set_position(const nmea::GPSFix &now, std::string link = ""); 
    void set_interval(uint32_t interval);
    
    bool tx_base_message(nr_v2x_mng *sock);  
    bool rx_platooning_message(const std::string &payload);
 
    uint64_t convert_timestamp_format(uint64_t usec = 0);

private:
     
    std::string build_lv_msg();
    std::string build_fv_msg();

    bool build_ssov_header(DB_V2X_T &dst);
    bool build_can_data(DB_V2X_PLATOONING_CAN &dst);  
    bool build_dev_info(DB_V2X_DEV_INFO_T &dst, uint32_t id= 0, uint16_t hw= 0, uint16_t sw= 0 , uint64_t timestamp= 0, uint64_t latency= 0);
 
    bool progress_rx_lv_msg(const DB_V2X_PLATOONING_LV_T &msg);
    bool progress_rx_fv_msg(const DB_V2X_PLATOONING_FV_T &msg); 
 
    uint8_t roll_type = PLATOONING_ROLL_TYPE_T::PLATOONING_ROLL_TYPE_FV;
    uint32_t psid = nr_v2x_psid_list_t::eV2XMSG_TYPE_5G_EM_V2V;
  
    struct vehicle_info_t{ 
        std::mutex lock;
        std::string vehicle_id = "KIAPI-VEH"; 
        std::string vehicle_number = "000K0000";  

        nmea::GPSFix fix;
        std::string link_id = "A2207G001101"; 

        uint32_t interval = 100;

        uint32_t seq = 0;
        uint32_t cnt = 0;
        
        uint32_t dev_id = 10000001;
    }info;

    struct rf_info_t
    {    
        uint32_t freq = 5915;
        uint32_t power = 20;
        uint32_t bw = 20;
        uint32_t scs = 0;
        uint32_t mcs = 0;
    } rf;
 
    bool debug = false; 
};
 

#endif