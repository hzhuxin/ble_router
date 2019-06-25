/*!
 *    @file  pbuf.c
 *   @brief  The pbuf module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/18/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "pbuf.h"
// #include "loader.h"
#include "util.h"
#include "hal_rtc.h"
// #include "bt.h"
#include "debug.h"
//#include "base64.h"
#include "config.h"
#include "option.h"
#include "record.h"
#include "behavior.h"

#include "protobuf-c/protobuf-c.h"
#include "IdentityMsg.pb-c.h"
#include "Register.pb-c.h"
#include "parameter.pb-c.h"
#include "settingandparam.pb-c.h"
#include "Setting.pb-c.h"
#include "Define.pb-c.h"
// #include "Download.pb-c.h"
#include "SimpleRsp.pb-c.h"
#include "GPS.pb-c.h"
#include "Env.pb-c.h"
#include "Behavior2.pb-c.h"
#include "Cellular.pb-c.h"
#include "Status.pb-c.h"
// #include "Sensor.pb-c.h"
#include "Debug.pb-c.h"
#include "App.pb-c.h"
#include "hal_cpu.h"
#include "app_task.h"
#include "net/cell.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_I);

/* Private variables ---------------------------------------------------------*/
static Protocol__IdentityMsg idMsg = PROTOCOL__IDENTITY_MSG__INIT;
static PB_PackageTypeDef pbPkg;
static void* pBuf[40];

/* Private functions ---------------------------------------------------------*/
// Wrapper for malloc
static void* pb_alloc(void *allocator_data, size_t size) {
  return pvPortMalloc(size);
}
// Wrapper for free
static void pb_free(void *allocator_data, void *pointer) {
  vPortFree(pointer);
}
// Allocator for protobuf
static ProtobufCAllocator allocator = {
  .alloc = pb_alloc,
  .free = pb_free,
  .allocator_data = NULL,
};

static PB_PackageTypeDef* makePkg(int16_t type, int16_t len) {
  pbPkg.version = 1;
  pbPkg.manufacture = 1;
  pbPkg.type = type;
  pbPkg.len = len;
  pbPkg.dummy = 0;
  uint16_t crc = util_crc16(0xffff, (uint8_t*)&pbPkg, 0, util_offset(PB_PackageTypeDef, crc));
  pbPkg.crc = util_crc16(crc, pbPkg.data, 0, pbPkg.len);
  return &pbPkg;
}

// GPS info from record
static void gpsInfoFromRecord(Protocol__GPS* dst, Record_GpsData *src) {
  // CAUSION, you need init the object first
  memset(dst, 0, sizeof(*dst));
  protocol__gps__init(dst);
  dst->has_timestamp = true;
  dst->timestamp = src->time;
  dst->has_fixtime = true;
  dst->fixtime = src->fixtime;
  if(src->dimention) {
    dst->has_latitude = true;
    dst->latitude = src->latitude;
    dst->has_longitude = true;
    dst->longitude = src->longitude;
    dst->has_altitude = true;
    dst->altitude = src->altidude;
    dst->has_quality = true;
    dst->quality = 1; // TODO: fixme
    dst->has_course = true;
    dst->course = src->heading;
    dst->has_hacc = true;
    dst->hacc = src->hAccuracy;
    dst->has_vacc = true;
    dst->vacc = src->vAccuracy;
    dst->has_speed = true;
    dst->speed = src->speed;
    dst->has_satellitesused = true;
    dst->satellitesused = src->usedStar;
    dst->has_satellitesinview = true;
    dst->satellitesinview = src->viewStar;
  }
}

// ENV info from record
static void envInfoFromRecord(Protocol__Env* dst, Record_EnvData *src) {
  // CAUSION, you need init the object first
  memset(dst, 0, sizeof(*dst));
  protocol__env__init(dst);
  dst->has_timestamp = true;
  dst->timestamp = src->time;
  dst->has_batteryvoltage = true;
  dst->batteryvoltage = src->voltage;
  dst->has_batterypower = true;
  dst->batterypower = src->power;
  dst->has_innertemperature = true;
  dst->innertemperature = src->temperature;
  dst->has_innerhumidity = true;
  dst->innerhumidity = src->humidity;
  dst->has_innerpressure = true;
  dst->innerpressure = src->pressure;
  dst->has_ambientlight = true;
  dst->ambientlight = src->light;
}

// Cellular info from record
static void cellularInfoFromRecord(Protocol__Cellular* dst, CellData_t *src) {
  // CAUSION, you need init the object first
  memset(dst, 0, sizeof(*dst));
  protocol__cellular__init(dst);
  dst->has_timestamp = true;
  dst->timestamp = src->time;
  dst->has_batteryvoltage = true;
  dst->batteryvoltage = src->voltage;
  dst->has_temperature = true;
  dst->temperature = src->temperature;
  dst->has_errorflag = true;
  dst->errorflag = src->err_flag;
  dst->has_totaltime = true;
  dst->totaltime = src->tot_time;
  dst->has_hwttime = true;
  dst->hwttime = src->hwt_time;
  dst->has_simtime = true;
  dst->simtime = src->sim_time;
  dst->has_numtime = true;
  dst->numtime = src->num_time;
  dst->has_rssitime = true;
  dst->rssitime = src->csq_time;
  dst->has_registertime = true;
  dst->registertime = src->reg_time;
  dst->has_attachtime = true;
  dst->attachtime = src->att_time;
  dst->has_pdptime = true;
  dst->pdptime = src->pdp_time;
  dst->has_connectiontime = true;
  dst->connectiontime = src->con_time;
  dst->has_communicationtime = true;
  dst->communicationtime = src->com_time;
  dst->has_smstime = true;
  dst->smstime = src->sms_time;
  dst->has_signalstrength = true;
  dst->signalstrength = src->rssi;
  dst->has_biterrorrate = true;
  dst->biterrorrate = src->ber;
  dst->has_radioaccesstechnology = true;
  dst->radioaccesstechnology = src->rat;
  dst->has_networkoperator = true;
  dst->networkoperator = src->mnc;
  if(src->latitude != 2000000000 && src->longitude != 2000000000) {
    dst->has_longitude = true;
    dst->longitude = src->longitude;
    dst->has_latitude = true;
    dst->latitude = src->latitude;
  }
  dst->has_exitflag = true;
  dst->exitflag = (src->ext_ber << 24) | (src->ext_rssi << 16) | src->ext_flag;
}

// Option data from parameter
static void optFromParameter(OPT_TypeDef* dst, Protocol__Parameter *src, OPT_TypeDef* dft) {
  memcpy(dst, dft, sizeof(*dft));

  for(int i = 0; i < src->n_serverhost; i++) {
    if(src->serverhost && src->serverhost[i]) {
      strcpy((char*)dst->serverHosts[i], src->serverhost[i]);
    }
  }
  for(int i = 0; i < src->n_serverport; i++) {
    if(src->serverport) {
      dst->serverPorts[i] = src->serverport[i];
    }
  }

  for(int i = 0; i < src->n_smsalarmreceiverlist; i++) {
    if(src->smsalarmreceiverlist && src->smsalarmreceiverlist[i]) {
      strcpy((char*)dst->smsReceivers[i], src->smsalarmreceiverlist[i]);
    }
  }
  for(int i = 0; i < src->n_smswakeupwhitelist; i++) {
    if(src->smswakeupwhitelist && src->smswakeupwhitelist[i]) {
      strcpy((char*)dst->smsOperators[i], src->smswakeupwhitelist[i]);
    }
  }

  if(src->envsamplethreshold) {
    dst->envThreshold = src->envsamplethreshold;
  }
  if(src->behaviorsamplethreshold) {
    dst->bhvThreshold = src->behaviorsamplethreshold;
  }
  if(src->gpssamplethreshold) {
    dst->gpsThreshold = src->gpssamplethreshold;
  }
  if(src->communicationthreshold) {
    dst->netThreshold = src->communicationthreshold;
  }
}

// Config data from setting
static void cfgFromSetting(CFG_TypeDef* dst, OAD_CfgDef* oad, Protocol__Setting *src, CFG_TypeDef* dft) {
  memcpy(dst, dft, sizeof(*dft));

  if(src->envsamplemode != 1 || src->envsampleinterval > 0) {
    dst->envMode = src->envsamplemode;
    dst->envIntv = src->envsampleinterval;
  }

  if(src->behaviorsamplemode != 1 || src->behaviorsampleinterval > 0) {
    dst->bhvMode = src->behaviorsamplemode;
    dst->bhvIntv = src->behaviorsampleinterval;
  }

  if(src->gpssamplemode != 1 || src->gpssampleinterval > 0) {
    dst->gpsMode = src->gpssamplemode;
    dst->gpsIntv = src->gpssampleinterval;
  }

  // Use GprsType instead of GprsMode: you stupid idiot!
  if(src->communicationmode == 1 && src->communicationinterval > 0) { // 1 for interval
    dst->netMode = 1;
    dst->netIntv = src->communicationinterval;
  } else if(src->communicationmode == 2 && src->communicationtimetable != NULL) { // 2 for table
    int table = 0;
    int len = strlen(src->communicationtimetable);
    if(len > 24) {
      len = 24;
    }
    for(int i = 0; i < len; i++) {
      if(src->communicationtimetable[i] == '1') {
        table |= 1 << i;
      }
    }
    if(table != 0) {
      dst->netMode = 3;
      dst->netIntv = table;
    }
  }

  if(src->smsmode != 1 || src->smsinterval > 0) {
    dst->smsMode = src->smsmode;
    dst->smsIntv = src->smsinterval;
  }

  if(src->alarmmode == 0 || src->alarmmode == 1) {
    dst->warningMode = src->alarmmode;
  }
  if(src->resetdevice == 0 || src->resetdevice == 1) {
    dst->resetMode = src->resetdevice;
  }
}

// Get identity message
static Protocol__IdentityMsg* getIdMsg(void) {
  static uint32_t i = 1;

  idMsg.msgindex = i++;
  idMsg.msgtoken = xTaskGetTickCount() & 0xffff;
  idMsg.deviceid = hal_cpu_get_id_str(); // TODO: fixme

  uint16_t crc = util_crc16(0xffff, (uint8_t*)idMsg.deviceid, 0, strlen(idMsg.deviceid));
  crc = util_crc16(crc, (uint8_t*)&idMsg.msgtoken, 0, 2);

  idMsg.msgtoken |= crc << 16;
  return &idMsg;
}
//// Check identity message
//static bool checkIdMsg(Protocol__IdentityMsg *id) {
//  if(id == NULL || id->uuid.len != 12 || id->uuid.data == NULL) {
//    return false;
//  }

//  if(memcmp(Util_getDeviceID(), id->uuid.data, id->uuid.len)) {
//    return false;
//  }

//  uint16_t crc = Util_calculateCrc16(id->uuid.data, 0, id->uuid.len);
//  crc = Util_updateCrc16(crc, (uint8_t*)&id->token, 0, 2);

//  return (id->token >> 16) == crc;
//}
/**
 * @brief Get application package
 *
 * @return the package
 */
PB_PackageTypeDef* getAppReq(int32_t command) {
  Protocol__AppReq req = PROTOCOL__APP_REQ__INIT;

  req.iden = getIdMsg();
  if(command != 0) {
    req.has_command = true;
    req.command = command;
  }

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__app_req__pack_to_buffer(&req, &sb.base);

  return makePkg(PROTOCOL__HEADER_TYPE__TypeAppReq, len);
}

#if 0
/**
 * @brief Get sensor package
 *
 * @return the package
 */
PB_PackageTypeDef* getSensorReq(Record_SensorData* src) {
  Protocol__SensorReq req = PROTOCOL__SENSOR_REQ__INIT;

  req.iden = getIdMsg();
  req.has_timestamp = true;
  req.timestamp = src->time;
  req.has_batteryvoltage = true;
  req.batteryvoltage = src->voltage;
  req.has_temperature = true;
  req.temperature = src->temperature;
  req.has_humidity = true;
  req.humidity = src->humidity;
  req.has_pressure = true;
  req.pressure = src->pressure;
  req.has_light = true;
  req.light = src->luminance;
  req.has_accelerationx = true;
  req.accelerationx = src->acceleration_x;
  req.has_accelerationy = true;
  req.accelerationy = src->acceleration_y;
  req.has_accelerationz = true;
  req.accelerationz = src->acceleration_z;

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__sensor_req__pack_to_buffer(&req, &sb.base);

  pbPkg.type = PROTOCOL__HEADER_TYPE__TypeSensorReq;
  pbPkg.len = len;

  return &pbPkg;
}

/**
 * @brief Get debug package
 *
 * @return the package
 */
PB_PackageTypeDef* getDebugReq(char* msg) {
  Protocol__DebugReq req = PROTOCOL__DEBUG_REQ__INIT;

  req.iden = getIdMsg();
  req.message = msg;

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__debug_req__pack_to_buffer(&req, &sb.base);

  pbPkg.type = PROTOCOL__HEADER_TYPE__TypeDebugReq;
  pbPkg.len = len;

  return &pbPkg;
}

*
@brief Get log package

@return the package
PB_PackageTypeDef* getLogReq(uint8_t* data, int length) {
  Protocol__DebugReq req = PROTOCOL__DEBUG_REQ__INIT;

  req.iden = getIdMsg();
  req.has_data = true;
  req.data.data = (uint8_t*)data;
  req.data.len = length;

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__debug_req__pack_to_buffer(&req, &sb.base);

  pbPkg.type = PROTOCOL__HEADER_TYPE__TypeDebugReq;
  pbPkg.len = len;

  return &pbPkg;
}
#endif

/**
 * @brief Get register package
 *
 * @return the package
 */
PB_PackageTypeDef* getRegisterReq(uint32_t status, uint32_t voltage, CellData_t *cell) {
  Protocol__RegisterReq req = PROTOCOL__REGISTER_REQ__INIT;

  req.iden = getIdMsg();
  req.mac = hal_cpu_get_mac_str();
  req.has_devicetype = true;
  req.devicetype = DEV_TYPE;
  req.has_hardwareversion = true;
  req.hardwareversion = HW_VERSION;
  req.has_firmwareversion = true;
  req.firmwareversion = SW_VERSION;
  req.has_status = true;
  req.status = status;
  req.has_batteryvoltage = true;
  req.batteryvoltage = voltage;

  if(cell) {
    req.has_signalstrength = true;
    req.signalstrength = cell->rssi;
    req.has_biterrorrate = true;
    req.biterrorrate = cell->ber;
    req.has_radioaccesstechnology = true;
    req.radioaccesstechnology = cell->rat;
    req.has_networkoperator = true;
    req.networkoperator = cell->mnc;

    req.imsi = CELL_GetIMSI();
  }

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__register_req__pack_to_buffer(&req, &sb.base);


  return makePkg(PROTOCOL__HEADER_TYPE__TypeRegisterReq, len);
}

/**
 * @brief Get setting package
 *
 * @return the package
 */
PB_PackageTypeDef* getSettingAndParamReq(void) {
  Protocol__SettingAndParamReq req = PROTOCOL__SETTING_AND_PARAM_REQ__INIT;

  req.iden = getIdMsg();

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__setting_and_param_req__pack_to_buffer(&req, &sb.base);

  return makePkg(PROTOCOL__HEADER_TYPE__TypeSettingAndParamReq, len);
}

/**
 * @brief Get setting package
 *
 * @return the package
 */
PB_PackageTypeDef* getParameterReq(void) {
  Protocol__ParameterReq req = PROTOCOL__PARAMETER_REQ__INIT;

  req.iden = getIdMsg();

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__parameter_req__pack_to_buffer(&req, &sb.base);

  return makePkg(PROTOCOL__HEADER_TYPE__TypeParameterReq, len);
}

/**
 * @brief Get setting package
 *
 * @return the package
 */
PB_PackageTypeDef* getSettingReq(void) {
  Protocol__SettingReq req = PROTOCOL__SETTING_REQ__INIT;

  req.iden = getIdMsg();

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__setting_req__pack_to_buffer(&req, &sb.base);

  return makePkg(PROTOCOL__HEADER_TYPE__TypeSettingReq, len);
}

#if 0
/**
 * @brief Get download package
 *
 * @return the package
 */
PB_PackageTypeDef* getDownloadReq(char* token, uint32_t offset, uint32_t length) {
  Protocol__DownloadReq req = PROTOCOL__DOWNLOAD_REQ__INIT;

  req.iden = getIdMsg();
  req.fileid = token;
  req.has_begin = true;
  req.begin = offset;
  req.has_length = true;
  req.length = length;

  ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
  size_t len = protocol__download_req__pack_to_buffer(&req, &sb.base);

  pbPkg.type = PROTOCOL__HEADER_TYPE__TypeDownloadReq;
  pbPkg.len = len;

  return &pbPkg;
}
#endif

/**
 * @brief Get env package
 *
 * @return the package
 */
PB_PackageTypeDef* getEnvReq(uint32_t *nRecords) {
  Protocol__EnvReq req = PROTOCOL__ENV_REQ__INIT;
  req.iden = getIdMsg();
  req.envinfo = (Protocol__Env**)pBuf;
  req.n_envinfo = 0;
  // Records to upload
  for(int i = 0; i < sizeof(pBuf) / sizeof(pBuf[0]); i++) {
    Record_EnvData d;
    if(!Record_Get(RECORD_TYPE_ENV, i, (uint8_t*)&d)) {
      break;
    } else {
      Protocol__Env* r = allocator.alloc(allocator.allocator_data, sizeof(Protocol__Env));
      if(r == NULL) {
        break;
      } else {
        // Env record to env info
        envInfoFromRecord(r, &d);
        // Add to list
        req.envinfo[i] = r;
        req.n_envinfo++;
        // Check length
        uint32_t len = protocol__env_req__get_packed_size(&req);
        if(len > PB_MAX_PDU_SIZE) {
          req.n_envinfo--;
          allocator.free(allocator.allocator_data, r);
          break;
        }
      }
    }
  }
  // Any record to upload?
  if(req.n_envinfo > 0) {
    // Serialize
    ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
    size_t len = protocol__env_req__pack_to_buffer(&req, &sb.base);
    // Delete record objects
    for(int i = 0; i < req.n_envinfo; i++) {
      allocator.free(allocator.allocator_data, req.envinfo[i]);
    }
    // Number of records
    *nRecords = req.n_envinfo;
    return makePkg(PROTOCOL__HEADER_TYPE__TypeEnvReq, len);
  }

  return NULL;
}
/**
 * @brief Get gps package
 *
 * @return the package
 */
PB_PackageTypeDef* getGpsReq(uint32_t *nRecords) {
  Protocol__GPSReq req = PROTOCOL__GPSREQ__INIT;
  req.iden = getIdMsg();
  req.gpsinfo = (Protocol__GPS**)pBuf;
  req.n_gpsinfo = 0;
  // Records to upload
  for(int i = 0; i < 20; i++) {
    Record_GpsData d;
    if(!Record_Get(RECORD_TYPE_GPS, i, (uint8_t*)&d)) {
      break;
    } else {
      Protocol__GPS* r = allocator.alloc(allocator.allocator_data, sizeof(Protocol__GPS));
      if(r == NULL) {
        break;
      } else {
        // Gps record to gps info
        gpsInfoFromRecord(r, &d);
        // Add to list
        req.gpsinfo[i] = r;
        req.n_gpsinfo++;
        // Check length
        uint32_t len = protocol__gpsreq__get_packed_size(&req);
        if(len > PB_MAX_PDU_SIZE) {
          req.n_gpsinfo--;
          allocator.free(allocator.allocator_data, r);
          break;
        }
      }
    }
  }
  // Any record to upload?
  if(req.n_gpsinfo > 0) {
    // Serialize
    ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
    size_t len = protocol__gpsreq__pack_to_buffer(&req, &sb.base);
    // Delete record objects
    for(int i = 0; i < req.n_gpsinfo; i++) {
      allocator.free(allocator.allocator_data, req.gpsinfo[i]);
    }
    // Number of records
    *nRecords = req.n_gpsinfo;
    return makePkg(PROTOCOL__HEADER_TYPE__TypeGPSReq, len);
  }

  return NULL;
}

/**
 * @brief Get Behavior package
 *
 * @return the package
 */
PB_PackageTypeDef* getBehaviorReq(uint32_t *nRecords) {
  Protocol__Behavior2Req req = PROTOCOL__BEHAVIOR2_REQ__INIT;
  req.iden = getIdMsg();
  req.behaviorinfo = (Protocol__Behavior2**)pBuf;
  req.n_behaviorinfo = 0;
  // Records to upload
  for(int i = 0; i < sizeof(pBuf) / sizeof(pBuf[0]); i++) {
    Record_ActData d;
    if(!Record_Get(RECORD_TYPE_ACT, i, (uint8_t*)&d)) {
      break;
    } else {
      Protocol__Behavior2* r = allocator.alloc(allocator.allocator_data, sizeof(Protocol__Behavior2));
      if(r == NULL) {
        break;
      } else {
        // Init
        memset(r, 0, sizeof(*r));
        protocol__behavior2__init(r);
        // Behavior record to behavior info
        r->has_timestamp = true;
        r->timestamp = d.time;
#if DEV_TYPE == 101 && HW_VERSION == 2
        r->has_odbax = true;
        r->odbax = d.odba.x;
        r->has_odbay = true;
        r->odbay = d.odba.y;
        r->has_odbaz = true;
        r->odbaz = d.odba.z;
        r->has_meandlx = true;
        r->meandlx = d.meandl.x;
        r->has_meandly = true;
        r->meandly = d.meandl.y;
        r->has_meandlz = true;
        r->meandlz = d.meandl.z;
#else
        r->has_odba = true;
        r->odba = d.odba;
#endif
        // Add to list
        req.behaviorinfo[i] = r;
        req.n_behaviorinfo++;
        // Check length
        uint32_t len = protocol__behavior2_req__get_packed_size(&req);
        if(len > PB_MAX_PDU_SIZE) {
          req.n_behaviorinfo--;
          allocator.free(allocator.allocator_data, r);
          break;
        }
      }
    }
  }
  // Any record to upload?
  if(req.n_behaviorinfo > 0) {
    // Serialize
    ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
    size_t len = protocol__behavior2_req__pack_to_buffer(&req, &sb.base);
    // Delete record objects
    for(int i = 0; i < req.n_behaviorinfo; i++) {
      allocator.free(allocator.allocator_data, req.behaviorinfo[i]);
    }
    // Number of records
    *nRecords = req.n_behaviorinfo;
    return makePkg(PROTOCOL__HEADER_TYPE__TypeBehavior2Req, len);
  }

  return NULL;
}

/**
 * @brief Get cellular package
 *
 * @return the package
 */
PB_PackageTypeDef* getCellularReq(uint32_t *nRecords) {
  Protocol__CellularReq req = PROTOCOL__CELLULAR_REQ__INIT;
  req.iden = getIdMsg();
  req.cellularinfo = (Protocol__Cellular**)pBuf;
  req.n_cellularinfo = 0;
  // Records to upload
  for(int i = 0; i < sizeof(pBuf) / sizeof(pBuf[0]); i++) {
    CellData_t d;
    if(!Record_Get(RECORD_TYPE_NET, i, (uint8_t*)&d)) {
      break;
    } else {
      Protocol__Cellular* r = allocator.alloc(allocator.allocator_data, sizeof(Protocol__Cellular));
      if(r == NULL) {
        break;
      } else {
        // Cell record to cellular info
        cellularInfoFromRecord(r, &d);
        // Add to list
        req.cellularinfo[i] = r;
        req.n_cellularinfo++;
        // Check length
        uint32_t len = protocol__cellular_req__get_packed_size(&req);
        if(len > PB_MAX_PDU_SIZE) {
          req.n_cellularinfo--;
          allocator.free(allocator.allocator_data, r);
          break;
        }
      }
    }
  }
  // Any record to upload?
  if(req.n_cellularinfo > 0) {
    // Serialize
    ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
    size_t len = protocol__cellular_req__pack_to_buffer(&req, &sb.base);
    // Delete record objects
    for(int i = 0; i < req.n_cellularinfo; i++) {
      allocator.free(allocator.allocator_data, req.cellularinfo[i]);
    }
    // Number of records
    *nRecords = req.n_cellularinfo;
    return makePkg(PROTOCOL__HEADER_TYPE__TypeCellularReq, len);
  }

  return NULL;
}

/**
 * @brief Get status package
 *
 * @return the package
 */
PB_PackageTypeDef* getStatusReq(uint32_t *nRecords) {
  Protocol__StatusReq req = PROTOCOL__STATUS_REQ__INIT;
  req.iden = getIdMsg();
  req.statusinfo = (Protocol__Status**)pBuf;
  req.n_statusinfo = 0;
  // Records to upload
  for(int i = 0; i < sizeof(pBuf) / sizeof(pBuf[0]); i++) {
    Record_ErrData d;
    if(!Record_Get(RECORD_TYPE_ERR, i, (uint8_t*)&d)) {
      break;
    } else {
      Protocol__Status* r = allocator.alloc(allocator.allocator_data, sizeof(Protocol__Status));
      if(r == NULL) {
        break;
      } else {
        // Init
        memset(r, 0, sizeof(*r));
        protocol__status__init(r);
        // Error record to error info
        r->has_timestamp = true;
        r->timestamp = d.time;
        r->has_statustype = true;
        r->statustype = d.type;
        r->has_statusvalue = true;
        r->statusvalue = d.data;
        r->has_firmwareversion = true;
        r->firmwareversion = d.version;
        // Add to list
        req.statusinfo[i] = r;
        req.n_statusinfo++;
        // Check length
        uint32_t len = protocol__status_req__get_packed_size(&req);
        if(len > PB_MAX_PDU_SIZE) {
          req.n_statusinfo--;
          allocator.free(allocator.allocator_data, r);
          break;
        }
      }
    }
  }
  // Any record to upload?
  if(req.n_statusinfo > 0) {
    // Serialize
    ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
    size_t len = protocol__status_req__pack_to_buffer(&req, &sb.base);
    // Delete record objects
    for(int i = 0; i < req.n_statusinfo; i++) {
      allocator.free(allocator.allocator_data, req.statusinfo[i]);
    }
    // Number of records
    *nRecords = req.n_statusinfo;
    return makePkg(PROTOCOL__HEADER_TYPE__TypeStatusReq, len);
  }

  return NULL;
}

#if 0
/**
 * @brief Get sms package
 *
 * @return the package
 */
PB_PackageTypeDef* getSmsReq(uint32_t rssi) {
  Protocol__GPSReq req = PROTOCOL__GPSREQ__INIT;
  req.iden = getIdMsg();
  req.has_sms = true;
  req.sms = 1;
  req.gpsinfo = (Protocol__GPS**)pBuf;
  req.n_gpsinfo = 0;

  int total = Record_Count(RECORD_TYPE_ENV);
  // Records to upload
  for(int i = total; i > 0; i--) {
    if(req.n_gpsinfo >= sizeof(pBuf) / sizeof(pBuf[0])) {
      break;
    }
    Record_EnvData d;
    if(!Record_Get(RECORD_TYPE_ENV, i - 1, (uint8_t*)&d)) {
      break;
    } else if(req.n_gpsinfo == 0 || d.dimention) {
      Protocol__GPS* r = allocator.alloc(allocator.allocator_data, sizeof(Protocol__GPS));
      if(r == NULL) {
        break;
      } else {
        memset(r, 0, sizeof(*r));
        protocol__gps__init(r);
        // Env record to gps info
        r->has_timestamp = true;
        r->timestamp = d.time;
        if(d.dimention) {
          r->has_latitude = true;
          r->latitude = d.latitude;
          r->has_longitude = true;
          r->longitude = d.longitude;
        }
        if(req.n_gpsinfo == 0) {
          r->has_temperature = true;
          r->temperature = d.temperature;
          r->has_humidity = true;
          r->humidity = d.humidity;
          r->has_light = true;
          r->light = d.luminance;
          r->has_pressure = true;
          r->pressure = d.pressure;
          r->has_batteryvoltage = true;
          r->batteryvoltage = d.voltage;
          r->has_signalstrength = true;
          r->signalstrength = rssi;
        }
        // Add to list
        req.gpsinfo[req.n_gpsinfo++] = r;
        // Check length
        uint32_t len = protocol__gpsreq__get_packed_size(&req);
        if(len > PB_MAX_SMS_SIZE) {
          req.n_gpsinfo--;
          allocator.free(allocator.allocator_data, r);
          break;
        }
      }
    }
  }
  // Any record to upload?
  if(req.n_gpsinfo > 0) {
    // Serialize
    ProtobufCBufferSimple sb  = PROTOBUF_C_BUFFER_SIMPLE_INIT(pbPkg.data);
    size_t len = protocol__gpsreq__pack_to_buffer(&req, &sb.base);
    // Delete record objects
    for(int i = 0; i < req.n_gpsinfo; i++) {
      allocator.free(allocator.allocator_data, req.gpsinfo[i]);
    }
    // Package header
    pbPkg.type = PROTOCOL__HEADER_TYPE__TypeGPSReq;
    pbPkg.len = len;
    return &pbPkg;
  }

  return NULL;
}
#endif

/* Global functions ----------------------------------------------------------*/
/**
 * @brief Get allocator
 *
 * @return the allocator
 */
ProtobufCAllocator* PB_Allocator(void) {
  return &allocator;
}

extern int BT_Send(int connid, char* data, int len);
extern int BT_Recv(int connid, char* data, int timeout);

/**
 * @brief Receive package
 *
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Receive(int connid, int timeout) {
  char* buf = (char*)&pbPkg;
  int offset = 0;
  TickType_t start = xTaskGetTickCount();

  if(connid == PB_BT_CONN_ID) {
    int len = BT_Recv(connid, (char*)&pbPkg, timeout);
    if(len < 0) {
      DBG_W("PB_Receive: connection error\r\n");
      return PB_ERR_CONN;
    } else if(len == 0) {
      DBG_W("PB_Receive: receive timeout\r\n");
      return PB_ERR_TIMEOUT;
    } else if(len != pbPkg.len + PB_HEADER_SIZE) {
      DBG_W("PB_Receive: header check failed\r\n");
      return PB_ERR_PACKAGE;
    } else {
      uint16_t crc = util_crc16(0xffff, (uint8_t*)&pbPkg, 0, util_offset(PB_PackageTypeDef, crc));
      crc = util_crc16(crc, pbPkg.data, 0, pbPkg.len);
      if(crc != pbPkg.crc) {
        DBG_W("PB_Receive: crc check failed\r\n");
        return PB_ERR_PACKAGE;
      } else {
        DBG_D("PB_Receive: succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
        return PB_ERR_OK;
      }
    }

    return PB_ERR_TIMEOUT;
  }

  while(!util_timeout(start, xTaskGetTickCount(), timeout)) {
    if(offset == 0) {
      DBG_V("PB_Receive: receiving header\r\n");
      offset = CELL_Recv(connid, buf, PB_HEADER_SIZE);
      if(offset < 0) {
        DBG_W("PB_Receive: connection error\r\n");
        return PB_ERR_CONN;
      } else if(offset == 0) {
        vTaskDelay(100);
      } else if(offset == PB_HEADER_SIZE) {
        DBG_D("PB_Receive: header received\r\n");
      }
    } else if(offset < PB_HEADER_SIZE) {
      DBG_W("PB_Receive: partial header(unlikely)\r\n");
      return PB_ERR_PACKAGE;
    } else if(offset < pbPkg.len + PB_HEADER_SIZE) {
      if(pbPkg.len > sizeof(pbPkg.data)) {
        DBG_W("PB_Receive: length overflow\r\n");
        return PB_ERR_PACKAGE;
      }
      DBG_V("PB_Receive: receiving data\r\n");
      int recv = CELL_Recv(connid, &buf[offset], pbPkg.len + PB_HEADER_SIZE - offset);
      if(recv < 0) {
        DBG_W("PB_Receive: connection error\r\n");
        return PB_ERR_CONN;
      }
      offset += recv;
      if(offset == pbPkg.len + PB_HEADER_SIZE) {
        DBG_D("PB_Receive: succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
        break;
      } else if(offset > pbPkg.len + PB_HEADER_SIZE) {
        DBG_W("PB_Receive: data overflow(unlikely)\r\n");
        return PB_ERR_PACKAGE;
      } else {
        vTaskDelay(100);
      }
    }
  }

  if(offset != pbPkg.len + PB_HEADER_SIZE) {
    DBG_W("PB_Receive: header check failed\r\n");
    return PB_ERR_TIMEOUT;
  }

  return PB_ERR_OK;
}

/**
 * @brief Execute an command
 *
 * @param connid
 * @param timeout in ticks
 * @param req
 *
 * @return true if success
 */
int PB_Command(int connid, int timeout, PB_PackageTypeDef* req) {
  int len = req->len + PB_HEADER_SIZE;
  DBG_D("PB_Command: sending request\r\n");
  int sent;

  if(connid == PB_BT_CONN_ID) {
    sent = BT_Send(connid, (char*)req, len);
  } else {
    sent = CELL_Send(connid, (char*)req, len);
  }

  if(sent < 0) {
    DBG_W("PB_Command: send failed(conn)\r\n");
    return PB_ERR_CONN;
  } else if(sent != len) {
    DBG_W("PB_Command: send failed(len)\r\n");
    return PB_ERR_PACKAGE;
  }
  int recv = PB_Receive(connid, timeout);
  if(recv < 0) {
    DBG_W("PB_Command: receive failed(%d)\r\n", recv);
    return recv;
  }
  return PB_ERR_OK;
}

/**
 * @brief Execute register command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Register(int connid, int timeout, int32_t status, int32_t voltage, CellData_t *cell, int32_t* pSN) {
  DBG_D("PB_Register: executing command\r\n");
  PB_PackageTypeDef* req = NULL;
  if(connid == PB_BT_CONN_ID) {
    req = getRegisterReq(status, voltage, NULL);
  } else {
    req = getRegisterReq(status, voltage, cell);
  }
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_W("PB_Register: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeRegisterRsp) {
    DBG_W("PB_Register: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__RegisterRsp *resp = protocol__register_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Register: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Register: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Register: result code error \r\n");
  } else {
    DBG_D("PB_Register: succeed\r\n");
    ret = PB_ERR_OK;
    // if(pSN != NULL && resp->sn > 0) {
    //   *pSN = resp->sn;
    // }
    // Type of the sim card (0-default, 1-uninet, 2-unim2m, 3-cmiot)
    // if(connid != BT_CONN_ID && resp->simtype == 2) {
    //   char *num = CELL_GetNUM();
    //   if(num != NULL) {
    //     if(memcmp(num, "4600102", strlen("4600102"))) {
    //       CELL_WriteNUM("4600102");
    //     }
    //   }
    // }
    // Synchronize time
    if(resp->timestamp && util_abs_diff(resp->timestamp, hal_rtc_get_time()) > 10) {
      hal_rtc_set_time(resp->timestamp);
      app_notify_sample_tasks(APP_TASK_EVT_RTC);
    }
  }
  protocol__register_rsp__free_unpacked(resp, &allocator);
  return ret;
}

#if 0
/**
 * @brief Execute confirm command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Confirm(int connid, int timeout) {
  DBG_D("PB_Confirm: executing command\r\n");
  PB_PackageTypeDef* req = getAppReq(1);
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Confirm: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeAppRsp) {
    DBG_W("PB_Confirm: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__AppRsp *resp = protocol__app_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Confirm: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_App: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Confirm: result code error \r\n");
  } else {
    DBG_I("PB_Confirm: succeed\r\n");
    ret = PB_ERR_OK;
  }
  protocol__app_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute sensor command
 *
 * @param connid
 * @param timeout in ticks
 * @param src: sensor data to send
 *
 * @return true if success
 */
int PB_Sensor(int connid, int timeout, Record_SensorData* src) {
  DBG_D("PB_Sensor: executing command\r\n");
  PB_PackageTypeDef* req = getSensorReq(src);
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Sensor: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeSensorRsp) {
    DBG_W("PB_Sensor: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Sensor: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Sensor: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Sensor: result code error \r\n");
  } else {
    DBG_I("PB_Sensor: succeed\r\n");
    ret = PB_ERR_OK;
  }
  protocol__simple_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute debug command
 *
 * @param connid
 * @param timeout in ticks
 * @param msg: message to send
 *
 * @return true if success
 */
int PB_Debug(int connid, int timeout, char* msg) {
  DBG_D("PB_Debug: executing command\r\n");
  PB_PackageTypeDef* req = getDebugReq(msg);
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Debug: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeDebugRsp) {
    DBG_W("PB_Debug: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Debug: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Debug: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Debug: result code error \r\n");
  } else {
    DBG_I("PB_Debug: succeed\r\n");
    ret = PB_ERR_OK;
  }
  protocol__simple_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute application command
 *
 * @param connid
 * @param timeout in ticks
 * @param link type: 1 - activate, 2 - upgrade
 * @param work mode: 0 - ongoing, 1 - done, 2 - cancel
 *
 * @return true if success
 */
int PB_App(int connid, int timeout, int32_t* linkType, int32_t* workMode) {
  DBG_D("PB_App: executing command\r\n");
  PB_PackageTypeDef* req = getAppReq(0);
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_App: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeAppRsp) {
    DBG_W("PB_App: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__AppRsp *resp = protocol__app_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_App: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_App: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_App: result code error \r\n");
  } else {
    DBG_I("PB_App: succeed\r\n");
    ret = PB_ERR_OK;
    if(linkType != NULL) {
      *linkType = resp->has_linktype ? resp->linktype : 0;
    }
    if(workMode != NULL) {
      *workMode = resp->has_workmode ? resp->workmode : 0;
    }
  }
  protocol__app_rsp__free_unpacked(resp, &allocator);
  return ret;
}
#endif

/**
 * @brief Execute setting and parameter command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_SettingAndParam(int connid, int timeout) {
  DBG_D("PB_SettingAndParam: executing command\r\n");
  PB_PackageTypeDef* req = getSettingAndParamReq();
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_SettingAndParam: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeSettingAndParamReq) {
    DBG_W("PB_SettingAndParam: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SettingAndParamRsp *resp = protocol__setting_and_param_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_SettingAndParam: unpack failed\r\n");
    return PB_ERR_PACKAGE;
  // } else if(!checkIdMsg(resp->iden)) {
  //   DBG_W("PB_SettingAndParam: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_SettingAndParam: result code error \r\n");
  } else {
    DBG_D("PB_SettingAndParam: succeed\r\n");
    ret = PB_ERR_OK;
    // Save parameter
    // if(connid != BT_CONN_ID || oadToken == NULL) {
      OAD_CfgDef* oad = NULL; // TODO: fixme
      CFG_TypeDef* cfg = allocator.alloc(allocator.allocator_data, sizeof(CFG_TypeDef));
      cfgFromSetting(cfg, oad, resp->settinginfo, CFG_Get());
      if(CFG_isChanged(cfg)) {
        CFG_Set(cfg);
        app_notify_sample_tasks(APP_TASK_EVT_CONFIG);
      }
      allocator.free(allocator.allocator_data, cfg);

      OPT_TypeDef* opt = allocator.alloc(allocator.allocator_data, sizeof(OPT_TypeDef));
      optFromParameter(opt, resp->parameterinfo, OPT_Get());
      OPT_Set(opt);
      allocator.free(allocator.allocator_data, opt);
    // }
  }
  protocol__setting_and_param_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute parameter command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Param(int connid, int timeout) {
  DBG_D("PB_Param: executing command\r\n");
  PB_PackageTypeDef* req = getParameterReq();
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Param: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeParameterRsp) {
    DBG_W("PB_Param: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__ParameterRsp *resp = protocol__parameter_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Param: unpack failed\r\n");
    return PB_ERR_PACKAGE;
  /* } else if(!checkIdMsg(resp->iden)) { */
  /*   DBG_W("PB_Param: identity check failed\r\n"); */
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Param: result code error \r\n");
  } else {
    DBG_D("PB_Param: succeed\r\n");
    ret = PB_ERR_OK;
    // Save parameter
    OPT_TypeDef* opt = allocator.alloc(allocator.allocator_data, sizeof(OPT_TypeDef));
    optFromParameter(opt, resp->parameterinfo, OPT_Get());
    OPT_Set(opt);
    allocator.free(allocator.allocator_data, opt);
  }
  protocol__parameter_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute setting command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Setting(int connid, int timeout, char* oadToken) {
  DBG_D("PB_Setting: executing command\r\n");
  PB_PackageTypeDef* req = getSettingReq();
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Setting: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeSettingRsp) {
    DBG_W("PB_Setting: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SettingRsp *resp = protocol__setting_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Setting: unpack failed\r\n");
    return PB_ERR_PACKAGE;
  // } else if(!checkIdMsg(resp->iden)) {
  //   DBG_W("PB_Setting: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Setting: result code error \r\n");
  } else {
    DBG_D("PB_Setting: succeed\r\n");
    ret = PB_ERR_OK;
    // Save settings
    // if(connid != BT_CONN_ID || oadToken == NULL) {
      CFG_TypeDef* cfg = allocator.alloc(allocator.allocator_data, sizeof(CFG_TypeDef));
      OAD_CfgDef* oad = NULL; // TODO: fixme
      cfgFromSetting(cfg, oad, resp->settinginfo, CFG_Get());
      if(CFG_isChanged(cfg)) {
        CFG_Set(cfg);
        app_notify_sample_tasks(APP_TASK_EVT_CONFIG);
      }
      allocator.free(allocator.allocator_data, cfg);
    // }
    // Save oad token
    // if(oadToken != NULL) {
    //   oadToken[0] = '\0';
    //   if(resp->firmwareid != NULL) {
    //     strcpy(oadToken, resp->firmwareid);
    //   }
    // }
  }
  protocol__setting_rsp__free_unpacked(resp, &allocator);
  return ret;
}

#if 0
/**
 * @brief Execute the oad download command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Download(int connid, int timeout, char* token, uint32_t offset, uint32_t length, Protocol__DownloadRsp** presp) {
  DBG_D("PB_Download: executing command\r\n");
  PB_PackageTypeDef* req = getDownloadReq(token, offset, length);
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Download: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeDownloadRsp) {
    DBG_W("PB_Download: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__DownloadRsp *resp = protocol__download_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Download: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Download: identity check failed\r\n");
//    protocol__download_rsp__free_unpacked(resp, &allocator);
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Download: result code error \r\n");
    protocol__download_rsp__free_unpacked(resp, &allocator);
  } else {
    DBG_I("PB_Download: succeed\r\n");
    *presp = resp;
    ret = PB_ERR_OK;
  }
  return ret;
}
#endif

/**
 * @brief Execute the env upload command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Env(int connid, int timeout) {
  uint32_t nRecords = 0;
  DBG_D("PB_Env: executing command\r\n");
  PB_PackageTypeDef* req = getEnvReq(&nRecords);
  if(req == NULL) {
    DBG_D("PB_Env: no more data\r\n");
    return PB_ERR_DATA;
  }
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Env: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeEnvRsp) {
    DBG_W("PB_Env: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Env: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Env: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Env: result code error \r\n");
  } else {
    DBG_D("PB_Env: succeed\r\n");
    ret = PB_ERR_OK;
    // if(connid != BT_CONN_ID) {
      Record_Delete(RECORD_TYPE_ENV, nRecords);
    // }
  }
  protocol__simple_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute the gps upload command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Gps(int connid, int timeout) {
  uint32_t nRecords = 0;
  DBG_D("PB_Gps: executing command\r\n");
  PB_PackageTypeDef* req = getGpsReq(&nRecords);
  if(req == NULL) {
    DBG_D("PB_Gps: no more data\r\n");
    return PB_ERR_DATA;
  }
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Gps: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeGPSRsp) {
    DBG_W("PB_Gps: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Gps: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Gps: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Gps: result code error \r\n");
  } else {
    DBG_D("PB_Gps: succeed\r\n");
    ret = PB_ERR_OK;
    // if(connid != BT_CONN_ID) {
      Record_Delete(RECORD_TYPE_GPS, nRecords);
    // }
  }
  protocol__simple_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute the behavior upload command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Bhv(int connid, int timeout) {
  uint32_t nRecords = 0;
  DBG_D("PB_Bhv: executing command\r\n");
  PB_PackageTypeDef* req = getBehaviorReq(&nRecords);
  if(req == NULL) {
    DBG_I("PB_Bhv: no more data\r\n");
    return PB_ERR_DATA;
  }
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Bhv: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeBehavior2Rsp) {
    DBG_W("PB_Bhv: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Bhv: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Bhv: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Bhv: result code error \r\n");
  } else {
    DBG_D("PB_Bhv: succeed\r\n");
    ret = PB_ERR_OK;
    Record_Delete(RECORD_TYPE_ACT, nRecords);
  }
  protocol__simple_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute the cellular upload command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Cellular(int connid, int timeout) {
  uint32_t nRecords = 0;
  DBG_D("PB_Cellular: executing command\r\n");
  PB_PackageTypeDef* req = getCellularReq(&nRecords);
  if(req == NULL) {
    DBG_I("PB_Cellular: no more data\r\n");
    return PB_ERR_DATA;
  }
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Cellular: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeCellularRsp) {
    DBG_W("PB_Cellular: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Cellular: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Cellular: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Cellular: result code error \r\n");
  } else {
    DBG_I("PB_Cellular: succeed\r\n");
    ret = PB_ERR_OK;
    Record_Delete(RECORD_TYPE_NET, nRecords);
  }
  protocol__simple_rsp__free_unpacked(resp, &allocator);
  return ret;
}

/**
 * @brief Execute the status upload command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Status(int connid, int timeout) {
  uint32_t nRecords = 0;
  DBG_D("PB_Status: executing command\r\n");
  PB_PackageTypeDef* req = getStatusReq(&nRecords);
  if(req == NULL) {
    DBG_I("PB_Status: no more data\r\n");
    return PB_ERR_DATA;
  }
  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Status: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeStatusRsp) {
    DBG_W("PB_Status: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Status: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Status: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Status: result code error \r\n");
  } else {
    DBG_D("PB_Status: succeed\r\n");
    ret = PB_ERR_OK;
    Record_Delete(RECORD_TYPE_ERR, nRecords);
  }
  protocol__simple_rsp__free_unpacked(resp, &allocator);
  return ret;
}

#if 0
/**
 * @brief Send SMS
 *
 * @return true if success
 */
int PB_SMS(int rssi) {
  PB_PackageTypeDef* req = getSmsReq(rssi);
  if(req == NULL) {
    return PB_ERR_DATA;
  }
  int ret = PB_ERR_OK;
  int len = req->len + Util_offset(PB_PackageTypeDef, data);
  if(len > 0) {
    char* out = allocator.alloc(allocator.allocator_data, 160);
    len = Base64_Encode((uint8_t*)&pbPkg, len, (int8_t*)out);
    if(!CELL_SMSSend((char*)CFG_Get()->smsNumber, out, len)) {
      ret = PB_ERR_SMS;
    }
    allocator.free(allocator.allocator_data, out);
  }
  return ret;
}

/**
 * @brief Execute factory command
 *
 * @param timeout in ticks
 *
 * @return true if success
 */
bool PB_Factory(int timeout, int32_t status, int32_t voltage) {
  PB_PackageTypeDef* req = getRegisterReq(status, voltage, NULL);
  char* rsp = NULL;
  int len = req->len + PB_HEADER_SIZE;
  DBG_D("PB_Factory: sending request\r\n");

  if(!DBG_printBase64((char*)req, len)) {
    DBG_W("PB_Factory: send failed\r\n");
    return false;
  } else if((rsp = DBG_gets(timeout)) == NULL) {
    DBG_I("PB_Factory: receive failed\r\n");
    return false;
  } else if(!Base64_Decode((int8_t*)rsp, strlen(rsp), (uint8_t*)&pbPkg)) {
    DBG_W("PB_Factory: base64 decode failed\r\n");
    return false;
  } else if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeRegisterRsp) {
    DBG_W("PB_Factory: response type wrong\r\n");
    return false;
  } else {
    bool ret = false;
    Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
    if(resp == NULL) {
      DBG_W("PB_Factory: unpack failed\r\n");
      return false;
//    } else if(!checkIdMsg(resp->iden)){
//      DBG_W("PB_Factory: identity check failed\r\n");
    } else if(resp->iden->rspcode != 0) {
      DBG_W("PB_Factory: result code error \r\n");
    } else {
      DBG_I("PB_Factory: succeed\r\n");
      ret = true;
    }
    protocol__simple_rsp__free_unpacked(resp, &allocator);
    return ret;
  }
}

/**
 * @brief Execute info command
 *
 * @param connid
 * @param sn
 *
 * @return true if success
 */
bool PB_Info(int connid, int sn) {
  PB_InfoTypeDef* req = (PB_InfoTypeDef*)&pbPkg;
  req->protocol = 1;
  req->dummy = xTaskGetTickCount() & 0xffff;
  req->type = DEV_TYPE;
  req->hver = HW_VERSION;
  Util_readDeviceID(req->uid);
  req->sn = sn;
  req->crc = Util_calculateCrc16((uint8_t*)req, 5, sizeof(PB_InfoTypeDef) - 9);
  req->crc = Util_updateCrc16(req->crc, (uint8_t*)req, 0, 5);
  req->chk = Util_updateCrc16(req->crc, req->uid, 3, sizeof(req->uid) - 3);

  DBG_D("PB_Info: sending request\r\n");
  if(!CELL_Send(connid, (char*)req, sizeof(PB_InfoTypeDef))) {
    DBG_W("PB_Info: send failed\r\n");
    return false;
  }
  if(!CELL_Waitack(connid)) {
    DBG_I("PB_Info: wait ack failed\r\n");
    return false;
  }
  return true;
}

/**
 * @brief Execute the log upload command
 *
 * @param connid
 * @param timeout in ticks
 *
 * @return true if success
 */
int PB_Log(int connid, int timeout, bool *more) {
  DBG_D("PB_Log: executing command\r\n");
  static uint8_t buf[PB_MAX_PDU_SIZE - 64];
  uint32_t nRecords = Record_Count(RECORD_TYPE_LOG);

  if(more) {
    *more = false;
  }

  if(nRecords <= 0) {
    DBG_I("PB_Log: no more data\r\n");
    return PB_ERR_DATA;
  }

  nRecords = Record_Read(RECORD_TYPE_LOG, buf, sizeof(buf));
  PB_PackageTypeDef* req = getLogReq(buf, nRecords);
  if(more) {
    *more = Record_Count(RECORD_TYPE_LOG) > nRecords;
  }

  int ret = PB_Command(connid, timeout, req);
  if(ret != PB_ERR_OK) {
    DBG_I("PB_Log: command failed\r\n");
    return ret;
  }

  if(pbPkg.type != PROTOCOL__HEADER_TYPE__TypeDebugRsp) {
    DBG_W("PB_Log: response type wrong\r\n");
    return PB_ERR_PACKAGE;
  }

  ret = PB_ERR_PACKAGE;
  Protocol__SimpleRsp *resp = protocol__simple_rsp__unpack(&allocator, pbPkg.len, pbPkg.data);
  if(resp == NULL) {
    DBG_W("PB_Log: unpack failed\r\n");
    return PB_ERR_PACKAGE;
//  } else if(!checkIdMsg(resp->iden)) {
//    DBG_W("PB_Log: identity check failed\r\n");
  } else if(resp->iden->rspcode != 0) {
    DBG_W("PB_Log: result code error \r\n");
  } else {
    DBG_I("PB_Log: succeed\r\n");
    Record_Delete(RECORD_TYPE_LOG, nRecords);
    ret = PB_ERR_OK;
  }
  protocol__simple_rsp__free_unpacked(resp, &allocator);
  return ret;
}
#endif
