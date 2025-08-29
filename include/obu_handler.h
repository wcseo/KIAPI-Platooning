/*
 * Project : KIAPI-Platooning
 * Author: WooChang Seo (wcseo@kiapi.or.kr) 
 * Date: 2025-08-29
 */ 

#ifndef _OBU_KETI_PLATOONING_OBU_HANDLER_H_
#define _OBU_KETI_PLATOONING_OBU_HANDLER_H_

#include <nr-v2x/nr_v2x.h> 
#include <cpp-framework/common/time.h>   
#include <dgd-sock/dgd_data_share_sock.h>
#include "platooning_service.h" 
#include "app_config.h"

class obu_handler : public nr_v2x_mng_handler, dgd_data_share_sock_client_event{

public: 
    obu_handler(app_config_t config);
    ~obu_handler();

    bool init();
    bool run(const std::string &ip , uint32_t port, uint32_t id , uint32_t tx_interval);
  
protected:
    bool on_dev_connection(nr_v2x_dev_info_t *dev, const std::string &ip, uint32_t port, bool connection);
    void on_rx_msg(nr_v2x_dev_info_t *dev, const v2x_parameter_field_t &param, const std::string &msg);
    void on_rx_msg_ext(nr_v2x_dev_info_t *dev, const v2x_parameter_field_t &param,
                       const std::vector<v2x_message_field_t> &msg,
                       const std::vector<nr_v2x_ext_status_msg_field_t> &status);
    void on_tx_msg(nr_v2x_dev_info_t *dev, const v2x_parameter_field_t &param, const std::string &msg);
    void on_tx_msg_ext(nr_v2x_dev_info_t *dev, const v2x_parameter_field_t &param,
                       const std::vector<v2x_message_field_t> &msg,
                       const std::vector<nr_v2x_ext_status_msg_field_t> &status);
    void on_ftp_conn_req(nr_v2x_dev_info_t *dev, uint32_t psid, uint8_t unit_id, uint32_t link_id);

    bool on_can_raw_data(uint32_t psid, io_struct &sock, const dgd_sock_msg_can_raw_data_t &msg, char *ptr);
    bool on_dgd_gps_data(uint32_t psid, io_struct &sock, const dgd_gps_data_t &msg);
    bool on_dgd_link_data(uint32_t psid, io_struct &sock, const geojson_link_data_msg_t &msg);
    bool on_dgd_cvib_data(uint32_t psid, io_struct &sock, const dgd_sock_msg_cvib_data_t &msg);


private: 
    nr_v2x_mng nr_sock;   

    int state_level = -1; // -1 not set, 0 ready, 1 active 
 
    thread_handler worker; 
    void progress_worker(void *arg);

    struct service_info_t
    {
        uint32_t dev_id = 0;
        uint32_t interval = 100;
        tick_timer tick;
    } srv;


    struct d2x_data_info_t{
        nmea::GPSFix fix;
        std::string link = "";
    }data;
 
    platooning_service platooning; 
    dgd_data_share_sock d2x;
    app_config_t config; 
};
 
#endif