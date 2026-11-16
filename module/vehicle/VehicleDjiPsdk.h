#pragma once
#include <atomic>
#include <string>

#include "VehicleBase.h"
#include "dji_flight_controller.h"
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
};
}  // namespace maicro
