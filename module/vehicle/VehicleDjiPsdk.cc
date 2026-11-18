#include "VehicleDjiPsdk.h"

#include <math.h>
#include <string.h>

#include "MaicroLogger.inl"
#include "conf/ConfM3Gimbal.h"
#include "dji_aircraft_info.h"
#include "dji_core.h"
#include "dji_fc_subscription.h"
#include "dji_gimbal_manager.h"
#include "dji_psdk/dji_psdk_api.h"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

namespace maicro {
std::atomic<uint8_t> VehicleDjiPsdk::s_flyToMissionState(0);
std::atomic<float32_t> VehicleDjiPsdk::s_flyToRemainDistance(0);
// 0xFF = 从未收到上报(哨兵), 语义同 kNetRtkUnknown;
// 这里用字面量而非引用该常量, 避免静态常量被 ODR-use 而缺类外定义导致链接失败。
std::atomic<uint8_t> VehicleDjiPsdk::s_netRtkState(0xFF);

void VehicleDjiPsdk::NetworkRtkStateCallback(
    E_DjiNetworkRtkOnboardState state) {
  // 回调运行在 SDK 收发线程, 只写原子量, 不做任何阻塞操作
  s_netRtkState.store((uint8_t)state);
  MAICRO_INFO("network rtk onboard state -> %d (2=transfer)", (int)state);
}

template <typename T>
static int64_t GetTopicValue(E_DjiFcSubscriptionTopic topic, T &value) {
  T_DjiDataTimestamp timestamp = {0};
  T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
      topic, (uint8_t *)&value, sizeof(T), &timestamp);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_DEBUG("get value of topic %d error, 0x%08llX", (int)topic,
                 returnCode);
    return -1;
  }
  return 0;
}

VehicleDjiPsdk::VehicleDjiPsdk() {}

VehicleDjiPsdk::~VehicleDjiPsdk() { DeInitVehicleObject(); }

int64_t VehicleDjiPsdk::InitVehicleObject(const ConfM3Gimbal *conf) {
  mainCameraMountPos_ = (E_MaicroMountPosition)conf->gimbalMountPos;

  if (InitVehiclePsdk(conf)) {
    MAICRO_ERROR("Init vehicle psdk failed...");
    return -1;
  }
  MAICRO_INFO("Init vehicle psdk success...");

  if (InitVehicleModule()) {
    MAICRO_ERROR("Init vehicle module failed...");
    return -2;
  }

  if (fcSubscriptionReady_ && InitVehicleSubscription()) {
    MAICRO_ERROR("Init vehicle subscription failed, flight control disabled");
    flightCtrlReady_ = false;
  }

  if (!flightCtrlReady_) {
    MAICRO_ERROR("flight control NOT ready, takeoff/landing/fly-to/go-home "
                 "unavailable, gimbal control still works");
  }
  return 0;
}

int64_t VehicleDjiPsdk::StartApplication(void) {
  T_DjiReturnCode returnCode = DjiCore_ApplicationStart();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("start sdk application error, 0x%08llX", returnCode);
    return -1;
  }
  MAICRO_INFO("start sdk application success");
  return 0;
}

void VehicleDjiPsdk::DeInitVehicleObject(void) {
  if (flightCtrlReady_) {
    DjiFlightController_AntiRegisterOpenMisInfoCallBack();
    DjiFlightController_DeInit();
    flightCtrlReady_ = false;
  }
  if (fcSubscriptionReady_) {
    DjiFcSubscription_DeInit();
    fcSubscriptionReady_ = false;
  }
  if (gimbalReady_) {
    DjiGimbalManager_Deinit();
    gimbalReady_ = false;
  }
}

/*******************************************************
 * 云台控制
 *******************************************************/
int64_t VehicleDjiPsdk::ResetGimbal(E_MaicroMountPosition pos) {
  return DjiGimbalManager_Reset(MountPositionConvert(pos),
                                DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
}

int64_t VehicleDjiPsdk::RotateGimbal(E_MaicroMountPosition pos, float32_t pitch,
                                     float32_t roll, float32_t yaw,
                                     float64_t time, bool isRelative) {
  T_DjiGimbalManagerRotation dji_rotation;
  memset(&dji_rotation, 0, sizeof(dji_rotation));
  dji_rotation.pitch = pitch;
  dji_rotation.roll = roll;
  dji_rotation.yaw = yaw;
  dji_rotation.time = time;
  dji_rotation.rotationMode = isRelative
                                  ? DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE
                                  : DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
  return DjiGimbalManager_Rotate(MountPositionConvert(pos), dji_rotation);
}

int64_t VehicleDjiPsdk::SetGimbalMode(E_MaicroMountPosition pos,
                                      E_MaicroGimbalMode gimbalMode) {
  T_DjiReturnCode returnCode = DjiGimbalManager_SetMode(
      MountPositionConvert(pos), (E_DjiGimbalMode)gimbalMode);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("set gimbal mode failed, 0x%08llX", returnCode);
  }
  return returnCode;
}

/******************************************************
 * 飞行控制
 *******************************************************/
int64_t VehicleDjiPsdk::StartFlightControl(void) {
  T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Obtain joystick authority failed, error code: 0x%08llX",
                 returnCode);
    return -1;
  }
  MAICRO_INFO("Obtain joystick authority success");
  return 0;
}

int64_t VehicleDjiPsdk::StopFlightControl(void) {
  T_DjiReturnCode returnCode =
      DjiFlightController_ReleaseJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Release joystick authority failed, error code: 0x%08llX",
                 returnCode);
    return -1;
  }
  MAICRO_INFO("Release joystick authority success");
  return 0;
}

int64_t VehicleDjiPsdk::FlightControlTakeoff(void) {
  T_DjiReturnCode returnCode = DjiFlightController_StartTakeoff();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Request to take off failed, error code: 0x%08llX",
                 returnCode);
    return -1;
  }
  return 0;
}

int64_t VehicleDjiPsdk::FlightControlLanding(void) {
  T_DjiReturnCode returnCode = DjiFlightController_StartLanding();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Start landing failed, error code: 0x%08llX", returnCode);
    return -1;
  }
  return 0;
}

int64_t VehicleDjiPsdk::FlightControlConfirmLanding(void) {
  T_DjiReturnCode returnCode = DjiFlightController_StartConfirmLanding();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Start confirm landing failed, error code: 0x%08llX",
                 returnCode);
    return -1;
  }
  return 0;
}

int64_t VehicleDjiPsdk::FlightControlGoHome(void) {
  T_DjiReturnCode returnCode = DjiFlightController_StartGoHome();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Start go home failed, error code: 0x%08llX", returnCode);
    return -1;
  }
  return 0;
}

int64_t VehicleDjiPsdk::FlyToPosition(const S_MaicroPosition &target,
                                      float32_t minEnrouteHeight,
                                      uint8_t speed) {
  T_DjiFlightControllerStartMissionReq req;
  T_DjiFlightControllerStartMissionRsp rsp;
  memset(&req, 0, sizeof(req));
  memset(&rsp, 0, sizeof(rsp));

  req.version = 1;
  req.operation = 0;  // 新任务
  req.mea = minEnrouteHeight;
  req.fly_vel = speed;
  req.goal_num = 1;
  // PSDK 要求弧度
  req.cmd_mode_point_info[0].lat = DEG2RAD(target.latitude);
  req.cmd_mode_point_info[0].lon = DEG2RAD(target.longitude);
  req.cmd_mode_point_info[0].alt = target.altitude;

  MAICRO_INFO("fly to position: lat %.8f, lon %.8f, alt %.2f (ellipsoid), "
              "mea %.1f, speed %u",
              target.latitude, target.longitude, target.altitude,
              minEnrouteHeight, speed);

  T_DjiReturnCode returnCode = DjiFlightController_SetModeStartMission(req, &rsp);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("start fly to mission failed, error code: 0x%08llX",
                 returnCode);
    return -1;
  }
  if (rsp.ret_code != 0) {
    MAICRO_ERROR("start fly to mission rejected, ret: %u, error code: %u",
                 rsp.ret_code, rsp.error_code);
    return -2;
  }
  MAICRO_INFO("start fly to mission success, code name: %u", rsp.code_name);
  return 0;
}

int64_t VehicleDjiPsdk::SetVelocityControlBody(void) {
  // 参照 fangxian-M4 VehicleDjiPsdk::SetFlightCtrlMode(VELOCITY_BODY):
  // 水平/垂直速度控制 + 角速率 + 机体坐标系 + 稳定模式
  T_DjiFlightControllerJoystickMode joystickMode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
  };
  DjiFlightController_SetJoystickMode(joystickMode);
  MAICRO_INFO("set joystick velocity control mode, body coordinate");
  return 0;
}

int64_t VehicleDjiPsdk::SetVelocityControlGround(void) {
  // 地面坐标系 NEU: x 北为正, y 东为正. RTK返航按北向/东向误差闭环飞回
  T_DjiFlightControllerJoystickMode joystickMode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
  };
  DjiFlightController_SetJoystickMode(joystickMode);
  MAICRO_INFO("set joystick velocity control mode, ground coordinate");
  return 0;
}

int64_t VehicleDjiPsdk::FeedVelocityVector(float32_t x, float32_t y,
                                           float32_t z, float32_t yaw) {
  T_DjiReturnCode returnCode;

  // 限幅, 平移保持小速度: 水平 3 m/s, 垂直 2 m/s, 航向 30 deg/s
  float32_t speedX = fabs(x) < 3 ? x : (x > 0 ? 3 : -3);
  float32_t speedY = fabs(y) < 3 ? y : (y > 0 ? 3 : -3);
  float32_t speedZ = fabs(z) < 2 ? z : (z > 0 ? 2 : -2);
  float32_t rateYaw = fabs(yaw) < 30 ? yaw : (yaw > 0 ? 30 : -30);

  T_DjiFlightControllerJoystickCommand joystickCommand = {speedX, speedY,
                                                           speedZ, rateYaw};
  returnCode = DjiFlightController_ExecuteJoystickAction(joystickCommand);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_DEBUG("execute joy stick action failed, error code: 0x%08X",
                 returnCode);
    return -1;
  }

  return 0;
}

/*******************************************************
 * 状态查询
 *******************************************************/
int64_t VehicleDjiPsdk::QueryDroneStatus(E_MaicroFlightStatus &status) {
  T_DjiFcSubscriptionFlightStatus flightStatus = 0;
  if (GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, flightStatus)) {
    return -1;
  }
  status = (E_MaicroFlightStatus)flightStatus;
  return 0;
}

int64_t VehicleDjiPsdk::QueryDroneDisplayMode(uint8_t &mode) {
  T_DjiFcSubscriptionDisplaymode displayMode = 0;
  if (GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                    displayMode)) {
    return -1;
  }
  mode = displayMode;
  return 0;
}

int64_t VehicleDjiPsdk::QueryDronePosition(S_MaicroPosition &position) {
  T_DjiFcSubscriptionPositionFused positionFused;
  memset(&positionFused, 0, sizeof(positionFused));
  if (GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, positionFused)) {
    return -1;
  }
  position.latitude = RAD2DEG(positionFused.latitude);
  position.longitude = RAD2DEG(positionFused.longitude);
  position.altitude = positionFused.altitude;
  position.satelliteNum = positionFused.visibleSatelliteNumber;
  return 0;
}

int64_t VehicleDjiPsdk::QueryDroneRelativeHeight(float32_t &height) {
  T_DjiFcSubscriptionAltitudeFused altitudeFused = 0;
  T_DjiFcSubscriptionAltitudeOfHomePoint homePointAltitude = 0;
  if (GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                    homePointAltitude) ||
      GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, altitudeFused)) {
    return -1;
  }
  height = altitudeFused - homePointAltitude;
  return 0;
}

int64_t VehicleDjiPsdk::QueryDroneHeightFusion(float32_t &height) {
  T_DjiFcSubscriptionHeightFusion heightFusion = 0;
  if (GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION, heightFusion)) {
    return -1;
  }
  height = heightFusion;
  return 0;
}

int64_t VehicleDjiPsdk::QueryGimbalAngles(S_MaicroGimbalAngles &angles) {
  T_DjiFcSubscriptionGimbalAngles gimbalAngles;
  memset(&gimbalAngles, 0, sizeof(gimbalAngles));
  if (GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, gimbalAngles)) {
    return -1;
  }
  angles.pitch = gimbalAngles.x;
  angles.roll = gimbalAngles.y;
  angles.yaw = gimbalAngles.z;
  return 0;
}

int64_t VehicleDjiPsdk::QueryDroneYaw(float32_t &yawDeg) {
  // 四元数转欧拉角取 z 分量, 公式同 PSDK 官方 sample
  T_DjiFcSubscriptionQuaternion quat;
  memset(&quat, 0, sizeof(quat));
  if (GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, quat)) {
    return -1;
  }
  double q2sqr = (double)quat.q2 * quat.q2;
  double t0 = -2.0 * ((double)quat.q2 * quat.q2 + (double)quat.q3 * quat.q3) + 1.0;
  double t1 = 2.0 * ((double)quat.q1 * quat.q2 + (double)quat.q0 * quat.q3);
  double yawRad = atan2(t1, t0);
  yawDeg = (float32_t)(RAD2DEG(yawRad));
  if (yawDeg < 0) yawDeg += 360.0f;
  return 0;
}

int64_t VehicleDjiPsdk::QueryRtkSolution(uint8_t &solution) {
  T_DjiFcSubscriptionRtkPositionInfo info = 0;
  if (GetTopicValue(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, info)) {
    return -1;
  }
  solution = info;
  return 0;
}

/*******************************************************
 * 初始化设置
 *******************************************************/
int64_t VehicleDjiPsdk::InitVehiclePsdk(const ConfM3Gimbal *conf) {
  T_DjiReturnCode returnCode = DjiUser_PrepareSystemEnvironment();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    return -1;
  }

  T_DjiUserInfo userInfo;
  DjiUser_FillInUserInfo(&userInfo, conf);
  returnCode = DjiCore_Init(&userInfo);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("psdk core init error, 0x%08llX", returnCode);
    return -2;
  }

  DjiCore_SetAlias(conf->productAlias.c_str());
  // 控件配置变更后必须递增固件版本, 否则 Pilot 2 按版本号命中缓存不重新下载
  // v4: 新增喊话器 speaker 配置(语音指令 TTS 通道)
  // v5: 语音模块移出项目, 移除 speaker 配置
  T_DjiFirmwareVersion firmwareVersion = {5, 0, 0, 0};
  DjiCore_SetFirmwareVersion(firmwareVersion);
  DjiCore_SetSerialNumber(conf->serialNumber.c_str());

  T_DjiAircraftInfoBaseInfo baseInfo;
  memset(&baseInfo, 0, sizeof(baseInfo));
  returnCode = DjiAircraftInfo_GetBaseInfo(&baseInfo);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("get aircraft base info error, 0x%08llX", returnCode);
  } else {
    aircraftType_ = baseInfo.aircraftType;
    MAICRO_INFO("aircraft type: %d, mount position: %d", aircraftType_,
                (int)baseInfo.mountPosition);
  }
  MAICRO_INFO("psdk core init ok, alias: %s", conf->productAlias.c_str());
  return 0;
}

int64_t VehicleDjiPsdk::InitVehicleModule(void) {
  T_DjiReturnCode returnCode;

  // 初始化云台管理模块
  returnCode = DjiGimbalManager_Init();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("gimbal manager init failed, error code: 0x%08llX",
                 returnCode);
    return -1;
  }
  gimbalReady_ = true;
  MAICRO_INFO("gimbal manager init success");

  // 设置云台模式
  if (SetGimbalMode(mainCameraMountPos_, MAICRO_GIMBAL_MODE_FREE) ==
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_INFO("set gimbal manager mode to free success");
  }

  // 以下为飞控相关, 失败时不影响云台功能
  // 初始化FcSubscription
  returnCode = DjiFcSubscription_Init();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Init data subscription module error, stat:0x%08llX",
                 returnCode);
    return 0;
  }
  fcSubscriptionReady_ = true;
  MAICRO_INFO("Init data subscription module success");

  // 初始化FlightController
  T_DjiFlightControllerRidInfo ridInfo;
  memset(&ridInfo, 0, sizeof(ridInfo));
  returnCode = DjiFlightController_Init(ridInfo);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Init flight controller module error, stat:0x%08llX",
                 returnCode);
    return 0;
  }
  flightCtrlReady_ = true;
  MAICRO_INFO("flight controller module init success");

  // 设置JoystickCtrlAuthorityEvent回调
  returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
      FlightControlJoystickCtrlAuthSwitchEventCallback);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
    MAICRO_ERROR("Register joystick control authority event callback failed, "
                 "error code:0x%08llX",
                 returnCode);
  }

  // 飞往指定点任务状态回调
  returnCode =
      DjiFlightController_RegisterOpenMisInfoCallBack(FlyToMissionInfoCallback);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("Register fly to mission info callback failed, "
                 "error code:0x%08llX",
                 returnCode);
  }

  // 负载网络 RTK 状态回调: 仅用于诊断上报(RTK 命令回显)与可选硬门控。
  // 注册失败或机型不支持都不影响主流程 —— RTK 解状态(sol==50 固定解)才是权威判据,
  // 且若 RTK 源在遥控器/D-RTK2 而非负载自带网络RTK, 此回调可能永不触发,
  // 此时 s_netRtkState 保持哨兵值, QueryNetworkRtkState 返回 -1(不可用而非故障)。
  returnCode = DjiNetworkRtk_RegReceiveNetworkRtkStateCallback(
      NetworkRtkStateCallback);
  if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_INFO("network rtk state callback registered");
  } else if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
    MAICRO_WARN("register network rtk state callback failed, code 0x%08llX "
                "(diagnostic only, ignored)",
                returnCode);
  } else {
    MAICRO_INFO("network rtk not supported on this model (diagnostic only)");
  }

  return 0;
}

int64_t VehicleDjiPsdk::InitVehicleSubscription(void) {
  // required: 飞行控制依赖的话题, 订阅失败则禁用飞控
  struct {
    E_DjiFcSubscriptionTopic topic;
    E_DjiDataSubscriptionTopicFreq freq;
    const char *name;
    bool required;
  } topics[] = {
      {DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "flight status", true},
      {DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "display mode", true},
      {DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "height fusion", true},
      {DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "position fused", true},
      {DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "altitude fused", true},
      {DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
       DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, "altitude of home point", true},
      // 云台角度话题只支持 50Hz
      {DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
       DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, "gimbal angles", false},
      // 机头航向(四元数), 方向平移按键计算目标点用; 订阅失败时按键会提示重试
      {DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
       DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, "quaternion", false},
      // RTK 定位解状态, RTK返航按键校验用; 无RTK的机型订阅失败不影响其他功能
      {DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "rtk position info", false},
  };

  int64_t ret = 0;
  for (auto &t : topics) {
    T_DjiReturnCode returnCode =
        DjiFcSubscription_SubscribeTopic(t.topic, t.freq, NULL);
    if (returnCode == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE) {
      MAICRO_WARN("Subscribe topic %s duplicate", t.name);
    } else if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      MAICRO_ERROR("Subscribe topic %s failed, error code: 0x%08llX", t.name,
                   returnCode);
      if (t.required) ret = -1;
    } else {
      MAICRO_INFO("Subscribe topic %s success", t.name);
    }
  }
  return ret;
}

/*******************************************************
 * 内部回调
 *******************************************************/
T_DjiReturnCode VehicleDjiPsdk::FlightControlJoystickCtrlAuthSwitchEventCallback(
    T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData) {
  const char *owner = "unknown";
  switch (eventData.curJoystickCtrlAuthority) {
    case DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_RC:
      owner = "RC";
      break;
    case DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK:
      owner = "MSDK";
      break;
    case DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL:
      owner = "INTERNAL";
      break;
    case DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK:
      owner = "PSDK";
      break;
    default:
      break;
  }
  MAICRO_INFO("[Event] joystick ctrl authority -> %s, switch event: %d", owner,
              (int)eventData.joystickCtrlAuthoritySwitchEvent);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode VehicleDjiPsdk::FlyToMissionInfoCallback(
    T_DjiFlightControllerOpenMis eventData) {
  uint8_t lastState = s_flyToMissionState;
  s_flyToMissionState = eventData.mission_state_machine;
  s_flyToRemainDistance = eventData.distance_remaining;
  if (lastState != eventData.mission_state_machine) {
    MAICRO_INFO("[Event] fly to mission state %u -> %u, remain distance %.1f m, "
                "remain time %.1f s",
                lastState, eventData.mission_state_machine,
                eventData.distance_remaining, eventData.time_remaining);
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

E_DjiMountPosition VehicleDjiPsdk::MountPositionConvert(
    E_MaicroMountPosition position) {
  switch (position) {
    case MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1:
      return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
    case MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO2:
      return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2;
    case MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO3:
      return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3;
    case MAICRO_MOUNT_POSITION_EXTENSION_PORT:
      return DJI_MOUNT_POSITION_EXTENSION_PORT;
    default:
      return DJI_MOUNT_POSITION_UNKNOWN;
  }
}
}  // namespace maicro
