#include <algorithm>
#include <cmath>
#include "VehicleDjiPsdk.h"

#include <math.h>
#include <string.h>

#include <chrono>
#include <mutex>

#include "MaicroLogger.inl"
#include "conf/ConfM3Gimbal.h"
#include "dji_aircraft_info.h"
#include "dji_camera_manager.h"
#include "dji_core.h"
#include "dji_fc_subscription.h"
#include "dji_gimbal_manager.h"
#include "dji_psdk/dji_psdk_api.h"

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

namespace maicro {
namespace {
using TelemetryClock = std::chrono::steady_clock;

template <class T> struct TelemetrySlot {
  T data{};
  T_DjiDataTimestamp timestamp{};
  TelemetryClock::time_point received{};

  bool Fresh(TelemetryClock::time_point now) const {
    return received != TelemetryClock::time_point{} &&
           now - received <= std::chrono::milliseconds(600);
  }
};

std::mutex telemetryMutex;
TelemetrySlot<T_DjiFcSubscriptionFlightStatus> slotFlightStatus;
TelemetrySlot<T_DjiFcSubscriptionDisplaymode> slotDisplayMode;
TelemetrySlot<T_DjiFcSubscriptionPositionFused> slotPosition;
TelemetrySlot<T_DjiFcSubscriptionAltitudeFused> slotAltitude;
TelemetrySlot<T_DjiFcSubscriptionQuaternion> slotAttitude;
uint32_t landingCount = 0;
std::atomic<uint32_t> controlLossCount{0};

template <class T>
bool StoreTelemetrySlot(TelemetrySlot<T> &slot, const uint8_t *data, uint16_t bytes,
                     const T_DjiDataTimestamp *timestamp) {
  if (!data || !timestamp || bytes < sizeof(T))
    return false;
  if (slot.received != TelemetryClock::time_point{} &&
      slot.timestamp.millisecond == timestamp->millisecond &&
      slot.timestamp.microsecond == timestamp->microsecond)
    return false;
  memcpy(&slot.data, data, sizeof(T)); // 兼容 SDK 末尾带对齐填充的数据
  slot.received = TelemetryClock::now();
  slot.timestamp = *timestamp;
  return true;
}

template <class T, TelemetrySlot<T> *Slot>
T_DjiReturnCode OnTelemetryTopic(const uint8_t *data, uint16_t bytes,
                             const T_DjiDataTimestamp *timestamp) {
  std::lock_guard<std::mutex> lock(telemetryMutex);
  StoreTelemetrySlot(*Slot, data, bytes, timestamp);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode OnFlightStatusTopic(const uint8_t *data, uint16_t bytes,
                              const T_DjiDataTimestamp *timestamp) {
  std::lock_guard<std::mutex> lock(telemetryMutex);
  auto previous = slotFlightStatus.data;
  if (StoreTelemetrySlot(slotFlightStatus, data, bytes, timestamp) &&
      previous == MAICRO_FLIGHT_STATUS_IN_AIR &&
      (slotFlightStatus.data == MAICRO_FLIGHT_STATUS_ON_GROUND ||
       slotFlightStatus.data == MAICRO_FLIGHT_STATUS_STOPED)) {
    ++landingCount;
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
} // namespace

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

  // 云台探测必须在 DjiCore 初始化之后: 之前调用 DjiCameraManager_Init 必然返回
  // 未初始化错误(0xEE), 会把确实挂着相机的机型(如 M4E)误判成"无相机"并永久关掉云台。
  //
  // gimbal_mount_pos=0 -> 自动探测(推荐): 换机身/换插口都无需改配置。
  // 手动指定 1~5 -> 尊重配置, 但仍探测一次确认相机在位, 避免向空口发指令。
  if (mainCameraMountPos_ == MAICRO_MOUNT_POSITION_UNKNOWN) {
    MAICRO_INFO("gimbal_mount_pos=0, auto-detecting camera gimbal port...");
    mainCameraMountPos_ = DetectGimbalMountPos();
    gimbalAvailable_ = (mainCameraMountPos_ != MAICRO_MOUNT_POSITION_UNKNOWN);
  } else {
    MAICRO_INFO("gimbal_mount_pos=%d (manually configured), verifying camera "
                "presence...",
                (int)mainCameraMountPos_);
    E_MaicroMountPosition detected = DetectGimbalMountPos();
    if (detected == mainCameraMountPos_) {
      gimbalAvailable_ = true;
    } else if (detected != MAICRO_MOUNT_POSITION_UNKNOWN) {
      MAICRO_WARN("gimbal_mount_pos configured as %d but camera detected on "
                  "enum %d; using detected value",
                  (int)mainCameraMountPos_, (int)detected);
      mainCameraMountPos_ = detected;
      gimbalAvailable_ = true;
    } else if (gimbalProbeInconclusive_) {
      // 探测本身失败(相机管理模块初始化失败), 结果不可信: 手动配置的挂载口照常启用,
      // 否则 M4E 这类确实有相机的机型会因探测失败而永久失去云台控制。
      MAICRO_WARN("gimbal probe inconclusive, trusting configured mount pos %d",
                  (int)mainCameraMountPos_);
      gimbalAvailable_ = true;
    } else {
      MAICRO_WARN("gimbal_mount_pos configured as %d but no camera gimbal "
                  "detected on any port; gimbal control disabled",
                  (int)mainCameraMountPos_);
      gimbalAvailable_ = false;
    }
  }
  MAICRO_INFO("gimbal mount pos = %d, gimbal available = %s",
              (int)mainCameraMountPos_, gimbalAvailable_ ? "yes" : "no");

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
 * 前置条件: gimbalAvailable_ 为真(自动探测到相机, 或配置手动指定且在位)。
 * 机身没挂相机时直接返回失败并只警告一次, 避免每次按键都刷
 * 0x000000E1(模块不存在)错误日志, 也避免调用方误以为操作已生效。
 *******************************************************/
bool VehicleDjiPsdk::EnsureGimbalAvailable(const char *op) {
  if (gimbalAvailable_)
    return true;
  if (!gimbalNoCameraWarned_) {
    gimbalNoCameraWarned_ = true;
    MAICRO_WARN("no DJI camera gimbal present (probe found none); gimbal "
                "control disabled, first caller was %s. Flight control "
                "(takeoff/landing/move/go-home/track) is unaffected.",
                op ? op : "unknown");
  } else {
    MAICRO_DEBUG("gimbal %s skipped: no camera gimbal present",
                 op ? op : "unknown");
  }
  return false;
}

int64_t VehicleDjiPsdk::ResetGimbal(E_MaicroMountPosition pos) {
  if (!EnsureGimbalAvailable("recenter"))
    return -1;
  return DjiGimbalManager_Reset(MountPositionConvert(pos),
                                DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
}

int64_t VehicleDjiPsdk::RotateGimbal(E_MaicroMountPosition pos, float32_t pitch,
                                     float32_t roll, float32_t yaw,
                                     float64_t time, bool isRelative) {
  if (!EnsureGimbalAvailable("rotate"))
    return -1;
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
  if (!EnsureGimbalAvailable("set mode"))
    return -1;
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
  T_DjiReturnCode returnCode =
      DjiFlightController_ObtainJoystickCtrlAuthority();
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
  req.operation = 0; // 新任务
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

  T_DjiReturnCode returnCode =
      DjiFlightController_SetModeStartMission(req, &rsp);
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

int64_t VehicleDjiPsdk::FeedVelocity(const VelocityVector &v,
                                    const VelocityLimits &limits) {
  float32_t x = v.x, y = v.y, z = v.z, yaw = v.yaw;

  if (limits.requireFinite) {
    // 脚本飞行包络: 遥测异常可能算出 NaN/Inf, 整条拒绝下发。
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(yaw))
      return -1;
  }

  if (limits.vectorHorizontal) {
    // 合速度限幅: 超限时等比缩放 x/y, 保持航向不变。
    const double horizontal = std::hypot(x, y);
    if (horizontal > limits.horizontalMax) {
      const double scale = limits.horizontalMax / horizontal;
      x *= scale;
      y *= scale;
    }
    z = std::max(-limits.verticalMax, std::min(limits.verticalMax, z));
    yaw = std::max(-limits.yawRateMax, std::min(limits.yawRateMax, yaw));
  } else {
    // 逐轴独立限幅。**保持既有三元表达式写法**: fabs(x)<h 为假时取
    // (x>0 ? h : -h), 对 NaN 会落到 -h。这与"先判 isfinite 再钳位"
    // 不是等价变换, 既有平移/返航/航迹依赖此行为, 不在此擅自更改。
    const float32_t h = limits.horizontalMax;
    const float32_t zMax = limits.verticalMax;
    const float32_t yMax = limits.yawRateMax;
    x = fabs(x) < h ? x : (x > 0 ? h : -h);
    y = fabs(y) < h ? y : (y > 0 ? h : -h);
    z = fabs(z) < zMax ? z : (z > 0 ? zMax : -zMax);
    yaw = fabs(yaw) < yMax ? yaw : (yaw > 0 ? yMax : -yMax);
  }

  T_DjiFlightControllerJoystickCommand command = {x, y, z, yaw};
  const T_DjiReturnCode rc = DjiFlightController_ExecuteJoystickAction(command);
  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_DEBUG("execute joy stick action failed, error code: 0x%08X", rc);
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
  double t0 =
      -2.0 * ((double)quat.q2 * quat.q2 + (double)quat.q3 * quat.q3) + 1.0;
  double t1 = 2.0 * ((double)quat.q1 * quat.q2 + (double)quat.q0 * quat.q3);
  double yawRad = atan2(t1, t0);
  yawDeg = (float32_t)(RAD2DEG(yawRad));
  if (yawDeg < 0)
    yawDeg += 360.0f;
  return 0;
}

bool VehicleDjiPsdk::ReadTelemetry(VehicleTelemetry &out) {
  std::lock_guard<std::mutex> lock(telemetryMutex);
  const auto now = TelemetryClock::now();
  if (!slotFlightStatus.Fresh(now) || !slotDisplayMode.Fresh(now) ||
      !slotPosition.Fresh(now) || !slotAltitude.Fresh(now) ||
      !slotAttitude.Fresh(now))
    return false;

  const auto q = slotAttitude.data;
  const double norm = q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3;
  if (!std::isfinite(norm) || norm < 0.8 || norm > 1.2)
    return false;
  const double x = 1.0 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3);
  const double y = 2.0 * (q.q1 * q.q2 + q.q0 * q.q3);
  out.yawDeg = static_cast<float32_t>(RAD2DEG(atan2(y, x)));
  out.status = static_cast<E_MaicroFlightStatus>(slotFlightStatus.data);
  out.displayMode = slotDisplayMode.data;
  const auto pos = slotPosition.data;
  out.position = {RAD2DEG(pos.longitude), RAD2DEG(pos.latitude), pos.altitude,
                  pos.visibleSatelliteNumber};
  out.altitude = slotAltitude.data;
  out.landingCount = landingCount;
  return true;
}

uint32_t VehicleDjiPsdk::ControlLossCount() const {
  return controlLossCount.load();
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
  // 注册失败或机型不支持都不影响主流程 —— RTK 解状态(sol==50
  // 固定解)才是权威判据, 且若 RTK 源在遥控器/D-RTK2 而非负载自带网络RTK,
  // 此回调可能永不触发, 此时 s_netRtkState 保持哨兵值, QueryNetworkRtkState
  // 返回 -1(不可用而非故障)。
  returnCode =
      DjiNetworkRtk_RegReceiveNetworkRtkStateCallback(NetworkRtkStateCallback);
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
    DjiReceiveDataOfTopicCallback callback = nullptr;
  } topics[] = {
      {DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "flight status", true, OnFlightStatusTopic},
      {DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "display mode", true,
       OnTelemetryTopic<T_DjiFcSubscriptionDisplaymode, &slotDisplayMode>},
      {DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "height fusion", true},
      {DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "position fused", true,
       OnTelemetryTopic<T_DjiFcSubscriptionPositionFused, &slotPosition>},
      {DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "altitude fused", true,
       OnTelemetryTopic<T_DjiFcSubscriptionAltitudeFused, &slotAltitude>},
      {DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
       DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, "altitude of home point", true},
      // 云台角度话题只支持 50Hz
      {DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
       DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, "gimbal angles", false},
      // 机头航向(四元数), 方向平移按键计算目标点用; 订阅失败时按键会提示重试
      {DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
       "quaternion", false,
       OnTelemetryTopic<T_DjiFcSubscriptionQuaternion, &slotAttitude>},
      // RTK 定位解状态, RTK返航按键校验用; 无RTK的机型订阅失败不影响其他功能
      {DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
       DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "rtk position info", false},
  };

  int64_t ret = 0;
  for (auto &t : topics) {
    // 机身没有相机云台时订阅云台角度必然失败(0x000000E1 模块不存在),
    // 直接跳过并说明原因, 避免启动日志里出现看似故障的红色错误
    if (!gimbalAvailable_ &&
        t.topic == DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES) {
      MAICRO_INFO("skip subscribing %s: no camera gimbal present", t.name);
      continue;
    }
    T_DjiReturnCode returnCode =
        DjiFcSubscription_SubscribeTopic(t.topic, t.freq, t.callback);
    if (returnCode == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE) {
      MAICRO_WARN("Subscribe topic %s duplicate", t.name);
    } else if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      MAICRO_ERROR("Subscribe topic %s failed, error code: 0x%08llX", t.name,
                   returnCode);
      if (t.required)
        ret = -1;
    } else {
      MAICRO_INFO("Subscribe topic %s success", t.name);
    }
  }
  return ret;
}

/*******************************************************
 * 内部回调
 *******************************************************/
T_DjiReturnCode
VehicleDjiPsdk::FlightControlJoystickCtrlAuthSwitchEventCallback(
    T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData) {
  if (eventData.curJoystickCtrlAuthority !=
      DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK) {
    controlLossCount.fetch_add(1);
  }
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
    MAICRO_INFO(
        "[Event] fly to mission state %u -> %u, remain distance %.1f m, "
        "remain time %.1f s",
        lastState, eventData.mission_state_machine,
        eventData.distance_remaining, eventData.time_remaining);
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

E_DjiMountPosition
VehicleDjiPsdk::MountPositionConvert(E_MaicroMountPosition position) {
  switch (position) {
  case MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1:
    return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
  case MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO2:
    return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2;
  case MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO3:
    return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3;
  // E4: 必须映射到 EXTENSION_PORT_V2_NO4(值 4) 不是 EXTENSION_PORT(值 8)。
  // dji_fts.h 注明 M400 的 FTS 仅支持 EXTENSION_PORT_V2_NO4 也印证 M400
  // 的扩展口按 V2_NO 系列编号
  case MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO4:
    return DJI_MOUNT_POSITION_EXTENSION_PORT_V2_NO4;
  case MAICRO_MOUNT_POSITION_EXTENSION_PORT:
    return DJI_MOUNT_POSITION_EXTENSION_PORT;
  default:
    return DJI_MOUNT_POSITION_UNKNOWN;
  }
}

E_MaicroMountPosition VehicleDjiPsdk::DetectGimbalMountPos(void) {
  static const struct {
    E_MaicroMountPosition maicroPos;
    E_DjiMountPosition djiPos;
    const char *label;
  } kCandidates[] = {
      {MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1,
       DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, "E1"},
      {MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO2,
       DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2, "E2"},
      {MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO3,
       DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3, "E3"},
      {MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO4,
       DJI_MOUNT_POSITION_EXTENSION_PORT_V2_NO4, "E4"},
  };

  T_DjiReturnCode ret = DjiCameraManager_Init();
  gimbalProbeInconclusive_ = false;
  bool camMgrReady = (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
  if (!camMgrReady) {
    // 未初始化成功时内部句柄为空, 继续调用 GetCameraConnectStatus 会段错误
    gimbalProbeInconclusive_ = true;
    MAICRO_WARN("camera manager init failed 0x%08llX, skip gimbal probe",
                (unsigned long long)ret);
    return MAICRO_MOUNT_POSITION_UNKNOWN;
  }

  E_MaicroMountPosition found = MAICRO_MOUNT_POSITION_UNKNOWN;
  for (const auto &c : kCandidates) {
    bool connected = false;
    T_DjiReturnCode rc =
        DjiCameraManager_GetCameraConnectStatus(c.djiPos, &connected);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      MAICRO_DEBUG("probe %s: connect status query rc=0x%08llX (not present)",
                   c.label, (unsigned long long)rc);
      continue;
    }
    if (!connected) {
      MAICRO_DEBUG("probe %s: not connected", c.label);
      continue;
    }

    E_DjiCameraType camType = DJI_CAMERA_TYPE_UNKNOWN;
    T_DjiReturnCode tc = DjiCameraManager_GetCameraType(c.djiPos, &camType);
    MAICRO_INFO("probe %s: connected camera type=%d (rc=0x%08llX)", c.label,
                (int)camType, (unsigned long long)tc);
    if (tc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        camType == DJI_CAMERA_TYPE_PSDK) {
      MAICRO_INFO("probe %s: skipped, this is our own PSDK payload (not a "
                  "camera gimbal)",
                  c.label);
      continue;
    }
    if (camType == DJI_CAMERA_TYPE_UNKNOWN) {
      MAICRO_WARN("probe %s: connected but camera type unknown, accepting as "
                  "gimbal candidate",
                  c.label);
    }
    found = c.maicroPos;
    break;
  }

  if (found == MAICRO_MOUNT_POSITION_UNKNOWN) {
    MAICRO_INFO("gimbal probe: no DJI camera gimbal found on E1~E4");
  } else {
    const char *label = "?";
    if (found == MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1)
      label = "E1";
    else if (found == MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO2)
      label = "E2";
    else if (found == MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO3)
      label = "E3";
    else if (found == MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO4)
      label = "E4";
    MAICRO_INFO("gimbal probe: camera gimbal detected on %s (enum value %d)",
                label, (int)found);
  }
  if (camMgrReady)
    DjiCameraManager_DeInit();
  return found;
}
} // namespace maicro