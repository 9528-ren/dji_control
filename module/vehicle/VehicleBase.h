#pragma once
#include <string>

#include "MaicroTypedef.h"

namespace maicro {
class ConfM3Gimbal;

class VehicleBase {
 public:
  VehicleBase() {};
  virtual ~VehicleBase() {};

  // 初始化飞机: PSDK环境/核心/云台/数据订阅/飞控
  virtual int64_t InitVehicleObject(const ConfM3Gimbal *conf) = 0;

  // 启动PSDK应用, 需在所有模块(包括控件)注册完成后调用
  virtual int64_t StartApplication(void) = 0;

  virtual void DeInitVehicleObject(void) = 0;

  /*******************************************************
   * 云台控制
   *******************************************************/
  // 重置云台
  virtual int64_t ResetGimbal(E_MaicroMountPosition pos) = 0;

  // 旋转云台
  virtual int64_t RotateGimbal(E_MaicroMountPosition pos, float32_t pitch,
                               float32_t roll, float32_t yaw, float64_t time,
                               bool isRelative) = 0;

  virtual int64_t SetGimbalMode(E_MaicroMountPosition pos,
                                E_MaicroGimbalMode gimbalMode) = 0;

  /******************************************************
   * 飞行控制
   *******************************************************/
  // 飞控模块是否可用
  virtual bool IsFlightCtrlReady(void) = 0;
  // 开始飞行控制(获取控制权)
  virtual int64_t StartFlightControl(void) = 0;
  // 停止飞行控制(释放控制权)
  virtual int64_t StopFlightControl(void) = 0;
  // 起飞
  virtual int64_t FlightControlTakeoff(void) = 0;
  // 降落
  virtual int64_t FlightControlLanding(void) = 0;
  // 降落保护时确认降落
  virtual int64_t FlightControlConfirmLanding(void) = 0;
  // 返航
  virtual int64_t FlightControlGoHome(void) = 0;
  // 飞往指定点(飞控自主规划航线), target 经纬度单位为度, 高度为椭球高
  virtual int64_t FlyToPosition(const S_MaicroPosition &target,
                                float32_t minEnrouteHeight,
                                uint8_t speed) = 0;
  // 飞往指定点任务状态: 0 空闲, 其他为执行中
  virtual uint8_t QueryFlyToMissionState(void) = 0;
  // 飞往指定点剩余距离, m
  virtual float32_t QueryFlyToRemainDistance(void) = 0;
  // 切换为摇杆速度控制模式(机体坐标系, 稳定模式), 参照 fangxian-M4 写法
  virtual int64_t SetVelocityControlBody(void) = 0;
  // 切换为摇杆速度控制模式(地面坐标系 NEU: x北 y东, 稳定模式), RTK返航飞回用
  virtual int64_t SetVelocityControlGround(void) = 0;
  // 下发一次机体坐标系速度矢量 [vx 前后, vy 左右, vz 升降, yawRate]
  // 单位 m/s, deg/s. x/y 限幅 3, z 限幅 2, yaw 限幅 30
  // 需以约 10~50Hz 周期连续调用, 停止调用后飞机悬停
  virtual int64_t FeedVelocityVector(float32_t x, float32_t y, float32_t z,
                                     float32_t yaw) = 0;

  /*******************************************************
   * 状态查询
   *******************************************************/
  // 查询飞机状态
  virtual int64_t QueryDroneStatus(E_MaicroFlightStatus &status) = 0;
  // 查询飞行模式, 见 E_MaicroDisplayMode
  virtual int64_t QueryDroneDisplayMode(uint8_t &mode) = 0;
  // 查询飞机位置
  virtual int64_t QueryDronePosition(S_MaicroPosition &position) = 0;
  // 查询飞机相对起飞点高度
  virtual int64_t QueryDroneRelativeHeight(float32_t &height) = 0;
  // 查询飞机离地高度(融合)
  virtual int64_t QueryDroneHeightFusion(float32_t &height) = 0;
  // 查询云台角度
  virtual int64_t QueryGimbalAngles(S_MaicroGimbalAngles &angles) = 0;
  // 查询机头航向角, 单位: 度, 北为0顺时针[0,360)
  virtual int64_t QueryDroneYaw(float32_t &yawDeg) = 0;
  // 查询 RTK 定位解状态(50=窄区固定解), 见 E_DjiFcSubscriptionPositionSolutionProperty
  virtual int64_t QueryRtkSolution(uint8_t &solution) = 0;
  // 查询飞机类型
  int QueryAircraftType(void) { return aircraftType_; }

 protected:
  // 飞机类型, 见 E_DjiAircraftType
  int aircraftType_ = 0;
  // 主云台挂载位置, 从配置文件中读取
  E_MaicroMountPosition mainCameraMountPos_ =
      MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1;
};
}  // namespace maicro
