/*
 * Project : KIAPI-Platooning
 * Author: WooChang Seo (wcseo@kiapi.or.kr) 
 * Date: 2025-08-29
 */ 
   
#include "obu_handler.h"
#include <cpp-framework/common/log.h>
 
/// @brief OBU 통신부 handler Init
/// @param config OBU 통신부 및 서비스 Configure Params
obu_handler::obu_handler(app_config_t config) : nr_sock(nr_v2x_dev_type_t::V2X_DEV_TYPE_OBU, this),
worker(0,10),
platooning(),
d2x(false,config.dgd_ip)
{  
    this->config = config;

    d2x.set_container(this); 

    nr_sock.set_debug(config.verbose);
    platooning.set_debug(config.verbose); 
    platooning.set_device_id(config.dev_id);
    platooning.set_interval(config.tx_interval);
    platooning.set_vehicle_id(config.vc_id, config.vc_number);

    worker.start(this, &obu_handler::progress_worker, nullptr);
 
    bool res =  run(config.obu_ip, config.obu_port, config.dev_id, config.tx_interval);

    log_print("obu handler connect %s:%d , res = %d", config.obu_ip.c_str(), config.obu_port, res);

}

/// @brief 
obu_handler::~obu_handler()
{
 

}

/// @brief  OBU 통신부 handler dispose
/// @return 
bool obu_handler::init()
{ 
    std::vector<nr_v2x_psid_list_t>
        psids = {
           nr_v2x_psid_list_t::eV2XMSG_TYPE_5G_EM_V2V
        };
 
    for( auto it = psids.begin(); it != psids.end(); it++)
    {   
        printf("wsm service add %x \n",*it);

        while (nr_sock.request_wsm_service(0, v2x_action_type::V2X_ACTION_TYPE_ADD, *it) < 0)
        {
            printf("error, psid %x wsm service request failed \n", *it);
            sleep_for(1000);
        } 
    } 

    return true;
}


/// @brief OBU 통신부 서비스 Run 
/// @param ip 연결할 OBU 장치 IP
/// @param port 연결할 OBU 장치 TCP 포트
/// @param id Platooning 서비스 메시지 장치 ID 
/// @param tx_interval Platooning 서비스 메시지 전송주기
/// @return 실행 여부 (True or False)
bool obu_handler::run(const std::string &ip, uint32_t port , uint32_t id , uint32_t tx_interval)
{ 
    state_level = -1;
    srv.dev_id = id;
    srv.interval = tx_interval;
    srv.tick.set(tx_interval);

    return  nr_sock.set(false,ip,port); 
}


/// @brief OBU 장치 연결 이벤트 처리 Handler 
/// @param dev 연결 장치정보 
/// @param ip 장치 IP
/// @param port 연결 TCP 포트
/// @param connection 연결 여부(True : 연결 , False : 연결 해제)
/// @return don't care 
bool obu_handler::on_dev_connection(nr_v2x_dev_info_t *dev, const std::string &ip, uint32_t port, bool connection)
{
    if (connection)
    { 
        state_level = 0;
    }
    else
        state_level = -1;

    return false;
}

/// @brief 5G-NR RX 메시지 이벤트 처리 Handler
/// @param dev 연결 장치정보
/// @param param 5G-NR 수신 파라미터 정보
/// @param msg 5G-NR 수신 Payload 데이터  
void obu_handler::on_rx_msg(nr_v2x_dev_info_t *dev, const v2x_parameter_field_t &param, const std::string &msg)
{
    printf("debug : obu_handler::on_rx_msg \n");   
}

/// @brief 5G-NR Rx (Ext)메시지 이벤트 처리 Handler
/// @param dev 연결 장치정보
/// @param param 5G-NR 수신 파라미터 정보
/// @param msg 5G-NR 수신 메시지 필드 정보  
/// @param status 5G-NR 수신 Status 필드 정보
void obu_handler::on_rx_msg_ext(nr_v2x_dev_info_t *dev, const v2x_parameter_field_t &param,
                                const std::vector<v2x_message_field_t> &msg,
                                const std::vector<nr_v2x_ext_status_msg_field_t> &status)
{ 
    for (int i = 0; i < msg.size(); i++)
    { 
        //log_print("TX-EXT[%d] RES[%d] <%d> [%d]%s\n", i, param.res, msg[i].psid, msg[i].data.size(), string_to_hex(msg[i].data).c_str()); 
        if(msg[i].psid == nr_v2x_psid_list_t::EM_PT_SSOV){ 
            platooning.rx_platooning_message(msg[i].data); 
        } 
    } 
}

/// @brief 5G-NR Tx 메시지 전송 결과 이벤트 처리 Handler
/// @param dev 연결 장치정보
/// @param param 5G-NR 전송 파라미터 정보
/// @param msg 전송 메시지 Payload
void obu_handler::on_tx_msg(nr_v2x_dev_info_t *dev, const v2x_parameter_field_t &param, const std::string &msg)
{ 
    // don't care
}
 
/// @brief 5G-NR Tx (Ext)메시지 전송 결과 이벤트 처리 Handler
/// @param dev 연결 장치정보
/// @param param  5G-NR 전송 파라미터 정보
/// @param msg 전송 메시지 Payload 필드
/// @param status 전송 메시지 Status 필드 
void obu_handler::on_tx_msg_ext(nr_v2x_dev_info_t *dev, const v2x_parameter_field_t &param,
                                const std::vector<v2x_message_field_t> &msg,
                                const std::vector<nr_v2x_ext_status_msg_field_t> &status)
{ 
    // don't care  
}

/// @brief 5G-NR FTP 요청 이벤트 처리 Handler (RSU 연결 시 사용함)
/// @param dev 연결 장치 정보
/// @param psid 서비스 PSID
/// @param unit_id FTP 연결 정보 요청 UNIT ID 
/// @param link_id FTP 연결 정보 요청 LINK ID
void obu_handler::on_ftp_conn_req(nr_v2x_dev_info_t *dev, uint32_t psid, uint8_t unit_id, uint32_t link_id)
{
    // don't care   
} 

/// @brief CAN RAW 데이터 수신 이벤트 처리 Handler
/// @param psid 서비스 PSID
/// @param sock Sock-CAN ID 
/// @param msg CAN 메시지 
/// @param ptr CAN 장치 Extension 데이터 필드
/// @return 
bool obu_handler::on_can_raw_data(uint32_t psid, io_struct &sock, const dgd_sock_msg_can_raw_data_t &msg, char *ptr)
{
    // don't care 
    return true;
}

/// @brief D2X 장치 GPS 정보 수신 메시지 처리 이벤트 Handler
/// @param psid 서비스 PSID 
/// @param sock 소켓 정보 
/// @param msg GPS 정보 데이터 메시지 필드 
/// @return 
bool obu_handler::on_dgd_gps_data(uint32_t psid, io_struct &sock, const dgd_gps_data_t &msg)
{  
    data.fix.timestamp.setTime(msg.timestamp);
    data.fix.status = msg.status;   // Status: A=active, V=void (not locked)
    data.fix.type = msg.type;       // Type: 1=none, 2=2d, 3=3d
    data.fix.quality = msg.quality; // Quality: 
    data.fix.dilution = msg.dilution;                     // Combination of Vertical & Horizontal
    data.fix.horizontalDilution = msg.horizontalDilution; // Horizontal dilution of precision, initialized to 100, best =1, worst = >20
    data.fix.verticalDilution = msg.verticalDilution;     // Vertical is less accurate 
    data.fix.altitude = msg.altitude;       // meters
    data.fix.latitude = msg.latitude;       // degrees N
    data.fix.longitude = msg.longitude;     // degrees E
    data.fix.speed = msg.speed;             // km/h
    data.fix.travelAngle = msg.travelAngle; // degrees true north (0-360)
    data.fix.trackingSatellites = msg.trackingSatellites;
    data.fix.visibleSatellites = msg.visibleSatellites; 
    return true;
}

/// @brief 
/// @param psid 
/// @param sock 
/// @param msg 
/// @return 
bool obu_handler::on_dgd_link_data(uint32_t psid, io_struct &sock, const geojson_link_data_msg_t &msg)
{
    data.link = msg.id;
    return true;
}

/// @brief D2X 장치 Link ID 정보 수신 메시지 처리 이벤트 Handler
/// @param psid 서비스 PSID 
/// @param sock 소켓 정보
/// @param msg Link ID 메시지 
/// @return 
bool obu_handler::on_dgd_cvib_data(uint32_t psid, io_struct &sock, const dgd_sock_msg_cvib_data_t &msg)
{  
    // don't care
    return true;
}


/// @brief OBU 통신부 메인 서비스 Thread Progress 부 
/// @param arg nullptr
void obu_handler::progress_worker(void *arg)
{ 
    
    if(state_level  == 0){
        init(); // PSID 서비스 초기화  
        state_level = 1 ;
    }  
    
    if(state_level == 1)
    {         
        if(srv.tick.timeout())
        {
            platooning.set_position(data.fix, data.link);
            platooning.tx_base_message(&nr_sock);
        } 
    }else{ 
        sleep_for(100);
    }     
}