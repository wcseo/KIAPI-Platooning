/*
 * Project : KIAPI-Platooning
 * Author: WooChang Seo (wcseo@kiapi.or.kr) 
 * Date: 2025-08-29
 */ 

#include "platooning_service.h"
#include <nr-v2x/keti/db_v2x.h>
#include <cpp-framework/data/j2735.h>
#include <cpp-framework/common/log.h>
#include "utils.h" 
#include <arpa/inet.h>
#include <cpp-framework/common/endian.h>
 
#define DEBUG_PRINT 0


/// @brief 플래투닝 서비스 Init 
/// @param psid 플래투닝 V2X 서비스 PSID 
/// @param type 플래투닝 서비스 Roll Type 
platooning_service::platooning_service(uint32_t psid, uint8_t type )
{
    this->roll_type = type;
    this->psid = psid;
} 

/// @brief 플래투닝 서비스 dispose
platooning_service::~platooning_service()
{

}

/// @brief 디버깅 메시지 출력 설정 
/// @param set 활성 유무
void platooning_service::set_debug(bool set)
{
    debug = set;
}   

/// @brief 플래투닝 메시지 Device ID 설정
/// @param id 디바이스 ID 
void platooning_service::set_device_id(uint32_t id)
{
    info.dev_id = id;
}

/// @brief 플래투닝 메시지 Vehicle ID, 번호 설정
/// @param id 차량 ID 
/// @param number 차량 번호 
void platooning_service::set_vehicle_id(std::string id, std::string number)
{
    info.vehicle_id = id; 
    info.vehicle_number = number;
}

/// @brief 플래투닝 서비스 현재 위치 Update 
/// @param now 현재 위치 정보 
/// @param link 현재 Link ID 정보
void platooning_service::set_position(const nmea::GPSFix &now,std::string link)
{
    local_lock l(info.lock);
    info.fix = now;
    info.link_id = link;
}

/// @brief 플래투닝 메시지 송출 주기 설정 
/// @param interval 송출 주기(msec)
void platooning_service::set_interval(uint32_t interval)
{
    info.interval = interval;
}

/// @brief 플래투닝 서비스 메시지 전송
/// @param sock OBU 장치 연결 소켓 핸들러 
/// @return 전송 유무 
bool platooning_service::tx_base_message(nr_v2x_mng *sock)
{ 
    std::string msg;

    if (roll_type == PLATOONING_ROLL_TYPE_T::PLATOONING_ROLL_TYPE_LV)
    {
        msg = build_lv_msg(); // TODO 
    }
    else if (roll_type == PLATOONING_ROLL_TYPE_T::PLATOONING_ROLL_TYPE_FV)
    {
        msg = build_fv_msg();
    } 

    if (sock == nullptr)
    {
        log_print("platooning_service = tx error, sock ptr nullptr");
        return false;
    }  

    if(!sock->available())
    {
        log_print("platooning_service = tx error, sock not available");
        return false;
    }

    if(msg.size() == 0)
    {
        log_print("platooning_service = tx error, message build faild"); 
        return false;
    }
   
    std::string buf;
    buf.resize(sizeof(DB_V2X_T));
    DB_V2X_T *ssov = (DB_V2X_T *)buf.c_str();  
    build_ssov_header((DB_V2X_T &)*ssov);

    ssov->ulPayloadLength = BE<uint32_t>(msg.size()).value;

    // SSOV = DB_V2X_T + payload + CRC32   
    // Build. DB_V2X_T + Payload 
    buf.append(msg.c_str(), msg.size());
 
    // CRC32 = CRC 계산 범위는 DB_V2X_T 에서 페이로드까지 반영
    uint32_t crc = CLI_UTIL_GetCrc32((uint8_t *)buf.c_str(), buf.size());
    crc = htonl(crc);  

    buf.append((char *)&crc, sizeof(uint32_t)); 
    
    // Note. 외산 IP 의 경우, Payload 사이즈가 475 ~ 503으로 구성될 경우, 전송 X 
 
    bool res = sock->request_tx_extensible_msg(0, v2x_parameter_field_t(psid), EM_PT_SSOV, buf, true, 0xFF);
  
    return res;
}

/// @brief 플래투닝 메시지 수신 처리 
/// @param payload 플래투닝 메시지 Payload 
/// @return 성공 유무
bool platooning_service::rx_platooning_message(const std::string &payload)
{ 
    DB_V2X_T *ssov = (DB_V2X_T *)(payload.c_str());
  
    ssov->eServiceId = (DB_V2X_SERVICE_ID_E)(BE<uint32_t>(ssov->eServiceId).value >> 16);

    if(DEBUG_PRINT)
    {
        log_print("ssov->eDeviceType = %x", ssov->eDeviceType);
        log_print("ssov->eTeleCommType= %x", ssov->eTeleCommType);
        log_print("ssov->unDeviceId= %x", ssov->unDeviceId);
        log_print("ssov->ulTimeStamp= %llx", ssov->ulTimeStamp);
        log_print("ssov->eServiceId= %x", ssov->eServiceId);
        log_print("ssov->eActionType= %x", ssov->eActionType);
        log_print("ssov->eRegionId= %x", ssov->eRegionId);
        log_print("ssov->ePayloadType= %x", ssov->ePayloadType);
        log_print("ssov->eCommId= %x", ssov->eCommId);
        log_print("ssov->usDbVer= %x", ssov->usDbVer);
        log_print("ssov->usHwVer= %x", ssov->usHwVer);
        log_print("ssov->usSwVer= %x", ssov->usSwVer);
        log_print("ssov->ulPayloadLength= %x", ssov->ulPayloadLength);
        log_print("ssov->ulReserved= %x", ssov->ulReserved);
    }

    if (ssov->eServiceId != DB_V2X_SERVICE_ID_PLATOONING) 
    {
        log_print("platooning_service, error = \"ssov header service id[%d] != DB_V2X_SERVICE_ID_PLATOONING\"", ssov->eServiceId);
        return false;
    }

    DB_V2X_STATUS_TX_T *status = (DB_V2X_STATUS_TX_T *)(payload.c_str() + sizeof(DB_V2X_T));

    // status don't care
 
    DB_V2X_PLATOONING_T *header = (DB_V2X_PLATOONING_T *)(payload.c_str() + sizeof(DB_V2X_T) + sizeof(DB_V2X_STATUS_TX_T));

    char *ptr = (char*)(payload.c_str() + sizeof(DB_V2X_T) + sizeof(DB_V2X_STATUS_TX_T) + sizeof(DB_V2X_PLATOONING_T));
    
    bool res = false;
    switch (header->eDbV2XPtType)
    {
    case eDB_V2X_PT_TYPE_LV:
        res = progress_rx_lv_msg(*(DB_V2X_PLATOONING_LV_T *)ptr);
        break;
    case eDB_V2X_PT_TYPE_FV:
        res = progress_rx_fv_msg(*(DB_V2X_PLATOONING_FV_T *)ptr);
        break; 
    } 

    return res;
}

/// @brief LV 메시지 Build (TODO)
/// @return LV 메시지 바이너리 데이터 결과 
std::string platooning_service::build_lv_msg()
{
    std::string msg;  
    // V2X_STATUS_MSG
    // PLATOONING HEADER 
    // PLATONNING MSG  
    DB_V2X_STATUS_TX_T status; 

    //TODO 
 
    return msg;
}

/// @brief FV 메시지 Build 
/// @return FV 메시지 바이너리 데이터 결과 
std::string platooning_service::build_fv_msg()
{
    
    local_lock l(info.lock);

    std::string msg;
  
    // V2X_STATUS_MSG
    // PLATOONING HEADER 
    // PLATONNING MSG   
    DB_V2X_STATUS_TX_T tx_status;

    tx_status.ulReserved0 = 0;
    tx_status.ulReserved1 = 0; 

    build_dev_info(tx_status.stDbV2xDevL1);
    build_dev_info(tx_status.stDbV2xDevL2);
    build_dev_info(tx_status.stDbV2xDevL3, 0, 0, 0, get_epoch_time_msec(), 0);

    tx_status.unRxTargetDeviceId = 0; // ??
    tx_status.ucTxPwr = rf.power;
    tx_status.usTxFreq = rf.freq;
    tx_status.ucTxBw = rf.bw;
    tx_status.ucScs = rf.scs;
    tx_status.ucMcs = rf.mcs;
    tx_status.usTxRatio = info.interval;
    tx_status.stTxPosition.dTxLatitude = info.fix.latitude * 1000000000000.0; //J2735_DATA::Latitude::convert(info.fix.latitude);
    tx_status.stTxPosition.dTxLongitude = info.fix.longitude * 1000000000000.0; //J2735_DATA::Longitude::convert(info.fix.longitude);
    tx_status.stTxPosition.dTxAttitude = (double)J2735_DATA::Elevation::convert(info.fix.altitude);
    tx_status.unSeqNum = info.seq++;
    tx_status.unContCnt = (info.cnt++ % 101);
    tx_status.unTxVehicleSpeed = J2735_DATA::Velocity_MPS::convert(info.fix.speed * J2735_VELOCITY_KMS_TO_MS);
    tx_status.dTxVehicleHeading = info.fix.travelAngle;  //J2735_DATA::Heading::convert(info.fix.travelAngle);

    DB_V2X_PLATOONING_T header;

    header.eDbV2XPtType = eDB_V2X_PT_TYPE_FV;
    header.usV2xGroupId = 0;

    DB_V2X_PLATOONING_FV_T payload;

    memset(&payload,0x00,sizeof(DB_V2X_PLATOONING_FV_T));

    payload.eFvServiceId = eDB_V2X_PT_FV_SVC_ID_PLATOONING;
    payload.eFvMethodId = eDB_V2X_PT_FV_METHOD_ID_V2V;
    payload.unFvLength = 0 ;
    payload.usFvClientId = 0;
    payload.usFvSessionId = 0;
    payload.ucFvProtocolVer = 0;
    payload.ucFvInterfaceVer = 1;
    payload.eFvMsgType = eDB_V2X_PT_FV_MSG_TYPE_BROADCAST;
    payload.ucFvReturnCode = 0;
    payload.eFvVehicleType = eDB_V2X_PT_FV_VEHICLE_TYPE_UNKNOWN; 
    memcpy(&payload.szFvVehicleId, (char *)info.vehicle_id.c_str(),
           (info.vehicle_id.size() > (DB_V2X_PT_LV_VEHICLE_ID_LEN - 1)) ? DB_V2X_PT_LV_VEHICLE_ID_LEN - 1 : info.vehicle_id.size());

    memcpy(&payload.szFvVehicleNum, (char *)info.vehicle_number.c_str(),
           (info.vehicle_number.size() > (DB_V2X_PT_LV_VEHICLE_NUM_LEN - 1)) ? DB_V2X_PT_LV_VEHICLE_NUM_LEN - 1 : info.vehicle_number.size());

    payload.usFvMsgCount = 0;
    payload.eFvMsgId = eDB_V2X_PT_FV_MSG_ID_INVALID;
    payload.dFvLatitude = info.fix.latitude * 1000000000000.0;  //J2735_DATA::Latitude::convert(info.fix.latitude);    // double 변경 ?
    payload.dFvLongitude =  info.fix.longitude * 1000000000000.0; // J2735_DATA::Longitude::convert(info.fix.longitude); // double 변경 ?
    payload.usFvHeading = J2735_DATA::Heading::convert(info.fix.travelAngle);
    payload.usFvSpeed = J2735_DATA::Velocity_MPS::convert(info.fix.speed * J2735_VELOCITY_KMS_TO_MS);

    memcpy(&payload.szFvDriveLaneId, (char *)info.link_id.c_str(),
           (info.link_id.size() > (DB_V2X_PT_FV_LANE_LEN - 1)) ? DB_V2X_PT_FV_LANE_LEN - 1 : info.link_id.size());

    payload.eFvDriveStatus = eDB_V2X_PT_FV_DRIVE_STATUS_STAY_LANE;
    payload.eFvChangeCode = eDB_V2X_PT_FV_CHANGE_NO;
    payload.stFvPathPlan; // ??

    payload.usFvRecommDistance = 0;
    payload.usFvRecommSpeed = 0;

    build_can_data(payload.stFvCan);

    payload.unReserved1 = 0;
    payload.unReserved2 = 0;
    payload.unReserved3 = 0;
    payload.unReserved4 = 0;
    payload.unReserved5 = 0;
    payload.unReserved6 = 0;

    msg.append((char *)&tx_status, sizeof(DB_V2X_STATUS_TX_T));
    msg.append((char *)&header, sizeof(DB_V2X_PLATOONING_T));
    msg.append((char *)&payload, sizeof(DB_V2X_PLATOONING_FV_T));

    return msg;
}
 
/// @brief 플래투닝 서비스 메시지 DEV_INFO 데이터 Set
/// @param dst DB_V2X_INFO_T 메시지 규격
/// @param id 장치 ID
/// @param hw 하드웨어 Version
/// @param sw 소프트웨어 Version 
/// @param timestamp 현재 시간  
/// @param latency 지연 시간 
/// @return 
bool platooning_service::build_dev_info(DB_V2X_DEV_INFO_T &dst, uint32_t id, uint16_t hw, uint16_t sw, uint64_t timestamp, uint64_t latency)
{
    dst.unDevId = id;
    dst.usHwVer = hw;
    dst.usSwVer = sw;
    dst.ulTimeStamp = timestamp;
    dst.ulLatency = latency;

    return true;
}

/// @brief LV 메시지 수신 처리
/// @param msg 수신된 LV 메시지 
/// @return 처리 결과
bool platooning_service::progress_rx_lv_msg(const DB_V2X_PLATOONING_LV_T &msg)
{ 
    printf("platooning_service::progress_rx_lv_msg\n");

    printf("debug,[%s] LV[%s|%s] POS[%.7lf,%.7lf,%.3lf,%u]\n",
           get_time_string().c_str(),
           msg.szLvVehicleId, msg.szLvVehicleNum,
           msg.dLvLatitude * 0.000000000001, msg.dLvLongitude * 0.000000000001, msg.usLvHeading, msg.usLvSpeed);

    for (int i = 0; i < 10; i++)
    { 
        printf("\tstLvPathPlan[%d] = [%.7lf,%.7lf] \n",i,msg.stLvPathPlan.adLvLatitude[i] * 0.000000000001 ,msg.stLvPathPlan.adLvLongitude[i] * 0.000000000001);
    }

    return true;
} 

/// @brief FV 메시지 수신 처리
/// @param msg 수신된 FV 메시지 
/// @return 처리 결과
bool platooning_service::progress_rx_fv_msg(const DB_V2X_PLATOONING_FV_T &msg)
{   
    printf("platooning_service::progress_rx_fv_msg\n");

    return true;
}

/// @brief SSOV 프로토콜 Timestamp 포맷 규격 변환
/// @param usec Unix Tick Time 
/// @return SSOV Timestamp 포맷 데이터
uint64_t platooning_service::convert_timestamp_format(uint64_t usec)
{
    uint64_t timestamp = 0;

    timestamp = 2023032314344766828;
              //2025 08 18 14 22 24 00000 

    if(usec == 0)
    {
        usec = get_epoch_time_usec();
    }

    std::tm tm = get_tm();
  
    usec %= 1000000;
    usec /= 100;

    timestamp = (tm.tm_year + 1900) * 1000000000000000ll;
    timestamp += (tm.tm_mon + 1) * 10000000000000;
    timestamp += tm.tm_mday * 100000000000;
    timestamp += tm.tm_hour * 1000000000;
    timestamp += tm.tm_min * 10000000;
    timestamp += tm.tm_sec * 100000;
    timestamp += usec; 

    return timestamp;
}

/// @brief SSOV 헤더 메시지 데이터 적재 
/// @param dst 타겟 SSOV 헤더 
/// @return 성공 여부 , 항상 True 되며 차후 오류 검증 단계 추가 후 실패 시 False
bool platooning_service::build_ssov_header(DB_V2X_T &dst)
{   
    dst.eDeviceType = (DB_V2X_DEVICE_TYPE_E)BE<uint16_t>(DB_V2X_DEVICE_TYPE_OBU).value;
    dst.eTeleCommType = (DB_V2X_TELECOMMUNICATION_TYPE_E)BE<uint16_t>(DB_V2X_TELECOMM_TYPE_5G_PC5_BROADCAST).value;
    dst.unDeviceId = BE<uint32_t>(info.dev_id).value;
    dst.ulTimeStamp = BE<uint64_t>(convert_timestamp_format()).value; 
    dst.eServiceId = (DB_V2X_SERVICE_ID_E)BE<uint16_t>(DB_V2X_SERVICE_ID_PLATOONING).value;
    dst.eActionType =(DB_V2X_ACTION_TYPE_E) BE<uint16_t>(DB_V2X_ACTION_TYPE_REQUEST).value;
    dst.eRegionId = (DB_V2X_REGION_ID_E)BE<uint16_t>(DB_V2X_REGION_ID_DAEGU_KIAPI_PG).value; // TODO
    dst.ePayloadType = (DB_V2X_PAYLOAD_TYPE_E)BE<uint16_t>(DB_V2X_PAYLOAD_TYPE_PLATOONING).value;
    dst.eCommId = (DB_V2X_COMMUNCATION_ID_E)BE<uint16_t>(DB_V2X_COMM_ID_V2V).value;
    dst.usDbVer = BE<uint16_t>((DB_V2X_VERSION_MAJOR << CLI_DB_V2X_MAJOR_SHIFT) | DB_V2X_VERSION_MINOR).value;
    dst.usHwVer = BE<uint16_t>(0x01).value;
    dst.usSwVer = BE<uint16_t>(0x01).value;
    dst.ulPayloadLength = 0; // not set ;
    dst.ulReserved = 0;   
    return true;   
}

/// @brief 플래투닝 메시지 내 CAN 데이터 Set 
/// @param dst Target DB_V2X_PLATOONING_CAN 
/// @return Set 유무
bool platooning_service::build_can_data(DB_V2X_PLATOONING_CAN &dst)
{ 
    /* MSG ID 156 */
    dst.bEpsEnable = 0;
    dst.bOverrideIgnore = 0;
    dst.ucEpsSpeed = 0;
    dst.bAccEnable = 0;
    dst.bAebEnable = 0;
    dst.fAebDecelValue = 0;
    dst.ucAliveCnt = 0;
    /* MSG ID 157 */
    dst.fSteeringCmd = 0;
    dst.fAccelCmd = 0;
    /* MSG ID 710 */
    dst.bEpsEnStatus = 0;
    dst.ucEpsCtrlBdStatus = 0;
    dst.ucEpsCtrlStatus = 0;
    dst.ucOverrideStatus = 0;
    dst.fSteeringAngle = 0;
    dst.fSteeringDrvTq = 0;
    dst.fSteeringOutTq = 0;
    dst.ucEpsAliveCnt = 0;
    /* MSG ID 711 */
    dst.bAccEnStatus = 0;
    dst.ucAccCtrlBdStatus = 0;
    dst.ucAccVehErr = 0;
    dst.ucAccErr = 0;
    dst.ucAccUserCanErr = 0;
    dst.ucVehicleSpeed = 0;
    dst.fLongAccel = 0;
    dst.bTurnRightEn = 0;
    dst.bHazardEn = 0;
    dst.bTurnLeftEn = 0;
    dst.ucAccAliveCnt = 0;
    /* MSG ID 713 */
    dst.fLatAccel = 0;
    dst.fYawRate = 0;
    dst.fBrakeCylinder = 0;
    /* MSG ID 714 */
    dst.ucRadObjState = 0;
    dst.fRadObjLatPos = 0;
    dst.fRadObjDist = 0;
    dst.fRadObjRelSpd = 0;
    /* MSG ID 715 */
    dst.fAccPedalPos = 0;
    dst.unSteeringWheelAngleRate = 0;
    dst.ucBrakeActSignal = 0;

    return true;
}