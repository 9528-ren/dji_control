#pragma once
#include <stdint.h>

typedef float float32_t;
typedef double float64_t;

typedef enum {
  MAICRO_MOUNT_POSITION_UNKNOWN = 0,
  MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1 = 1,
  MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO2 = 2,
  MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO3 = 3,
  MAICRO_MOUNT_POSITION_EXTENSION_PORT = 4,
} E_MaicroMountPosition;

typedef enum {
  MAICRO_GIMBAL_MODE_FREE = 0,
  MAICRO_GIMBAL_MODE_FPV = 1,
  MAICRO_GIMBAL_MODE_YAW_FOLLOW = 2,
} E_MaicroGimbalMode;

typedef struct {
  float32_t pitch;
  float32_t roll;
  float32_t yaw;
} S_MaicroGimbalAngles;

typedef enum {
  MAICRO_FLIGHT_STATUS_STOPED = 0,     // 在地面, 电机未转
  MAICRO_FLIGHT_STATUS_ON_GROUND = 1,  // 在地面, 电机已转
  MAICRO_FLIGHT_STATUS_IN_AIR = 2,     // 空中
} E_MaicroFlightStatus;

// 飞行模式(取值与 E_DjiFcSubscriptionDisplayMode 一致, 只列出用到的)
typedef enum {
  MAICRO_DISPLAY_MODE_ATTITUDE = 1,
  MAICRO_DISPLAY_MODE_P_GPS = 6,
  MAICRO_DISPLAY_MODE_ASSISTED_TAKEOFF = 10,
  MAICRO_DISPLAY_MODE_AUTO_TAKEOFF = 11,
  MAICRO_DISPLAY_MODE_AUTO_LANDING = 12,
  MAICRO_DISPLAY_MODE_NAVI_GO_HOME = 15,
  MAICRO_DISPLAY_MODE_NAVI_SDK_CTRL = 17,
  MAICRO_DISPLAY_MODE_FORCE_AUTO_LANDING = 33,
  MAICRO_DISPLAY_MODE_ENGINE_START = 41,
} E_MaicroDisplayMode;

typedef struct {
  float64_t longitude;    // 经度, 单位: 度
  float64_t latitude;     // 纬度, 单位: 度
  float64_t altitude;     // 椭球高, 单位: m
  uint16_t satelliteNum;  // 可见卫星数
} S_MaicroPosition;
