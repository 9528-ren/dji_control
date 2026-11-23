#pragma once
#include <string>

#include "MaicroTypedef.h"
#include "vehicle/VehicleTypes.h"

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
                                float32_t minEnrouteHeight, uint8_t speed) = 0;
  // 飞往指定点任务状态: 0 空闲, 其他为执行中
  virtual uint8_t QueryFlyToMissionState(void) = 0;
  // 飞往指定点剩余距离, m
  virtual float32_t QueryFlyToRemainDistance(void) = 0;
  // 切换为摇杆速度控制模式(机体坐标系, 稳定模式), 参照 fangxian-M4 写法
  virtual int64_t SetVelocityControlBody(void) = 0;
  // 切换为摇杆速度控制模式(地面坐标系 NEU: x北 y东, 稳定模式), RTK返航飞回用
  virtual int64_t SetVelocityControlGround(void) = 0;
  /*******************************************************
   * 通用速度控制
   *******************************************************/
  /**
   * 下发一次速度矢量, 按 limits 指定的包络限幅。
   * 单位 m/s(x/y/z), deg/s(yaw)。需以约 10~50Hz 周期连续调用,
   * 停止调用后飞机悬停。
   *
   * 包络由调用方(业务层)决定: 按键平移/返航/航迹用 DefaultVelocityLimits(),
   * 闭环脚本飞行用 ScriptVelocityLimits()。硬件层只负责按包络钳位并下发,
   * 不认识任何具体业务。两种包络的水平限幅语义不同(逐轴 vs 合速度),
   * 详见 VelocityLimits 注释, 不可互换。
   */
  virtual int64_t FeedVelocity(const VelocityVector &v,
                               const VelocityLimits &limits) = 0;

  /**
   * 兼容入口: 等价于 FeedVelocity(v, DefaultVelocityLimits())。
   * 既有按键平移/返航/航迹调用点保持不变。
   */
  int64_t FeedVelocityVector(float32_t x, float32_t y, float32_t z,
                             float32_t yaw) {
    return FeedVelocity(VelocityVector{x, y, z, yaw}, DefaultVelocityLimits());
  }

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
  /*******************************************************
   * 通用遥测
   *******************************************************/
  /**
   * 一次性取齐一帧遥测快照, 要求每个话题都在新鲜度窗口内(见实现)。
   * 返回 false 表示数据不完整或过期, 调用方必须放弃本次决策而不是
   * 用陈旧值继续飞。任何需要闭环控制的业务都应使用本接口, 而不是
   * 分别调用下面那些单话题查询(它们各自取最新值, 时刻可能不一致)。
   */
  virtual bool ReadTelemetry(VehicleTelemetry &out) {
    (void)out;
    return false;
  }

  /**
   * 控制权丢失累计次数: 每当摇杆控制权从本载荷(PSDK)转走时递增。
   * 业务层在闭环开始前记下该值, 每轮循环比对, 一旦变化立即中止并刹停 ——
   * 这是防止"控制权已被遥控器抢走却还在下发速度"的核心保护。
   * 与具体业务无关, 任何闭环飞行都需要。
   */
  virtual uint32_t ControlLossCount() const { return 0; }

  // 查询 RTK 定位解状态(50=窄区固定解), 见
  // E_DjiFcSubscriptionPositionSolutionProperty
  virtual int64_t QueryRtkSolution(uint8_t &solution) = 0;
  // 查询负载网络 RTK 机载服务状态(见 E_DjiNetworkRtkOnboardState:
  // 2=DJI_TRANSFER 传输中)。 默认实现返回 -1 表示"该机型/该 RTK
  // 源不提供此状态", 子类可重写。 注意: 若 RTK 源是遥控器或 D-RTK2
  // 而非负载自带网络 RTK, 此状态可能恒为非传输态, 因此默认仅作诊断上报,
  // 只有配置 track_require_network_rtk=true 时才作硬门控。
  virtual int64_t QueryNetworkRtkState(uint8_t &state) {
    (void)state;
    return -1;
  }
  // 查询飞机类型
  int QueryAircraftType(void) { return aircraftType_; }

  /**
   * 主云台(大疆相机)实际所在的挂载口。
   * 配置 gimbal_mount_pos=0 时为启动时自动探测的结果; 手动指定时即指定值。
   * 返回 MAICRO_MOUNT_POSITION_UNKNOWN 表示机身上没有可用相机云台,
   * 调用方应据此跳过云台操作, 不要反复发指令刷错误日志。
   */
  E_MaicroMountPosition QueryGimbalMountPos(void) const {
    return mainCameraMountPos_;
  }

  /** 是否存在可控制的相机云台(自动探测或手动指定后确认在位) */
  bool IsGimbalAvailable(void) const { return gimbalAvailable_; }

protected:
  // 飞机类型, 见 E_DjiAircraftType
  int aircraftType_ = 0;
  // 主云台挂载位置。来源: 配置文件手动指定, 或配置为 0 时自动探测。
  // UNKNOWN 表示无可用相机(例如机身只挂了本载荷没挂相机), 此时云台功能关闭
  E_MaicroMountPosition mainCameraMountPos_ =
      MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1;
  // 云台是否真实可用。false 时所有云台调用立即返回失败并只警告一次,
  // 避免向不存在的模块刷 0x000000E1 错误日志
  bool gimbalAvailable_ = false;
};
} // namespace maicro