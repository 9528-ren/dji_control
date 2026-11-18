#pragma once
#include <atomic>
#include <string>

#include "VehicleBase.h"
#include "dji_flight_controller.h"
#include "dji_network_rtk.h"
#include "dji_typedef.h"

namespace maicro {
class VehicleDjiPsdk : public VehicleBase {
 public:
  VehicleDjiPsdk();
  ~VehicleDjiPsdk();

  int64_t InitVehicleObject(const ConfM3Gimbal *conf);

  int64_t StartApplication(void);

  void DeInitVehicleObject(void);

  /*******************************************************
   * 云台控制
   *******************************************************/
  int64_t ResetGimbal(E_MaicroMountPosition pos);

  int64_t RotateGimbal(E_MaicroMountPosition pos, float32_t pitch,
                       float32_t roll, float32_t yaw, float64_t time,
                       bool isRelative);

  int64_t SetGimbalMode(E_MaicroMountPosition pos,
                        E_MaicroGimbalMode gimbalMode);

  /******************************************************
   * 飞行控制
   *******************************************************/
  bool IsFlightCtrlReady(void) { return flightCtrlReady_; }
  int64_t StartFlightControl(void);
  int64_t StopFlightControl(void);
  int64_t FlightControlTakeoff(void);
  int64_t FlightControlLanding(void);
  int64_t FlightControlConfirmLanding(void);
  int64_t FlightControlGoHome(void);
  int64_t FlyToPosition(const S_MaicroPosition &target,
                        float32_t minEnrouteHeight, uint8_t speed);
  uint8_t QueryFlyToMissionState(void) { return s_flyToMissionState; }
  float32_t QueryFlyToRemainDistance(void) { return s_flyToRemainDistance; }
  int64_t SetVelocityControlBody(void);
  int64_t SetVelocityControlGround(void);
  int64_t FeedVelocityVector(float32_t x, float32_t y, float32_t z,
                             float32_t yaw);

  /*******************************************************
   * 状态查询
   *******************************************************/
  int64_t QueryDroneStatus(E_MaicroFlightStatus &status);
  int64_t QueryDroneDisplayMode(uint8_t &mode);
  int64_t QueryDronePosition(S_MaicroPosition &position);
  int64_t QueryDroneRelativeHeight(float32_t &height);
  int64_t QueryDroneHeightFusion(float32_t &height);
  int64_t QueryGimbalAngles(S_MaicroGimbalAngles &angles);
  int64_t QueryDroneYaw(float32_t &yawDeg);
  int64_t QueryRtkSolution(uint8_t &solution);

  /**
   * 负载网络 RTK 机载服务状态(见 E_DjiNetworkRtkOnboardState, 2=DJI_TRANSFER 传输中)。
   * 返回 -1 表示"从未收到回调上报", 即该机型不带负载网络RTK 或 RTK 源在别处
   * (遥控器/D-RTK2)。调用方不得把 -1 当作 RTK 故障。
   */
  int64_t QueryNetworkRtkState(uint8_t &state) {
    uint8_t v = s_netRtkState.load();
    if (v == kNetRtkUnknown) return -1;
    state = v;
    return 0;
  }

 private:
  /*******************************************************
   * 内部回调
   *******************************************************/
  // joystick控制权变更回调
  static T_DjiReturnCode FlightControlJoystickCtrlAuthSwitchEventCallback(
      T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);

  // 飞往指定点任务状态回调
  static T_DjiReturnCode FlyToMissionInfoCallback(
      T_DjiFlightControllerOpenMis eventData);

  // 负载网络 RTK 状态回调(只写原子量, 不做任何阻塞操作)
  static void NetworkRtkStateCallback(E_DjiNetworkRtkOnboardState state);

  // 挂载位置转换
  static E_DjiMountPosition MountPositionConvert(
      E_MaicroMountPosition position);

 private:
  /*******************************************************
   * 初始化设置
   *******************************************************/
  int64_t InitVehiclePsdk(const ConfM3Gimbal *conf);

  int64_t InitVehicleModule(void);

  int64_t InitVehicleSubscription(void);

 private:
  bool gimbalReady_ = false;
  bool fcSubscriptionReady_ = false;
  bool flightCtrlReady_ = false;

  static std::atomic<uint8_t> s_flyToMissionState;
  static std::atomic<float32_t> s_flyToRemainDistance;

  /** "从未收到网络RTK状态上报"的哨兵值(E_DjiNetworkRtkOnboardState 合法值为 0~6) */
  static const uint8_t kNetRtkUnknown = 0xFF;
  static std::atomic<uint8_t> s_netRtkState;
};
}  // namespace maicro
