#pragma once
#include "MaicroTypedef.h"

namespace maicro {

/**
 * 一帧飞机遥测快照(通用, 与任何具体业务无关)。
 *
 * 由硬件层在同一个锁内一次性取齐, 并保证每个话题都在新鲜度窗口内,
 * 避免业务层把不同时刻的状态/位置/姿态混着用。
 *
 * landingCount 是"空中->地面"跳变的累计次数。业务层用它判断
 * "我下发指令时飞机还在空中, 现在是否已经落地过", 从而拒绝在
 * 落地后继续执行旧的飞行脚本。任何飞行模式都需要这个判据。
 */
struct VehicleTelemetry {
  E_MaicroFlightStatus status = MAICRO_FLIGHT_STATUS_STOPED;
  uint8_t displayMode = 0;
  S_MaicroPosition position{};
  float32_t altitude = 0.0f;
  float32_t yawDeg = 0.0f;
  uint32_t landingCount = 0;
};

/**
 * 一次速度指令(通用)。x/y/z 单位 m/s, yaw 为航向角速率 deg/s。
 * 坐标系由调用前 SetVelocityControlBody()/SetVelocityControlGround() 决定:
 *   机体系: x 前正, y 右正
 *   地面系: x 北正, y 东正 (NEU)
 */
struct VelocityVector {
  float32_t x = 0.0f;
  float32_t y = 0.0f;
  float32_t z = 0.0f;
  float32_t yaw = 0.0f;
};

/**
 * 速度矢量限幅包络(通用)。
 *
 * **为什么必须显式区分两种包络, 不能合并成一个"取更严值"的函数:**
 * 两者的水平限幅语义根本不同 ——
 *   - 逐轴独立: |x|<=h 且 |y|<=h, 斜向合速度可达 h*sqrt(2)
 *   - 合速度  : hypot(x,y)<=h, 超限时按比例同时缩放 x/y, 保持方向
 * 把前者改成后者会让既有平移/返航/航迹的斜向速度变慢; 反之会让
 * 脚本飞行的斜向速度超出原设计上限。二者都不是等价变换。
 *
 * requireFinite 同理: 关闭时沿用既有三元表达式限幅(NaN 会被判为
 * 负向并钳到 -h), 打开时直接拒绝整条指令。这是两种不同的失效策略,
 * 由调用方按业务安全要求选择, 硬件层不擅自统一。
 */
struct VelocityLimits {
  /// 水平上限(m/s): vectorHorizontal=false 时为**每轴**上限, true 时为**合速度**上限
  float horizontalMax = 3.0f;
  /// 垂直速度上限(m/s), 对称限幅
  float verticalMax = 2.0f;
  /// 航向角速率上限(deg/s), 对称限幅
  float yawRateMax = 30.0f;
  /// true: 按合速度限幅并等比缩放 x/y(保方向); false: x/y 各自独立限幅
  bool vectorHorizontal = false;
  /// true: 任一轴为 NaN/Inf 时整条指令拒绝下发; false: 沿用逐轴钳位
  bool requireFinite = false;
};

/**
 * 默认包络: 按键平移 / 返航 / 航迹回放沿用的小速度限幅。
 * 逐轴 3 m/s, 垂直 2 m/s, 航向 30 deg/s, 不做有限性拒绝。
 */
inline VelocityLimits DefaultVelocityLimits() {
  VelocityLimits l;
  l.horizontalMax = 3.0f;
  l.verticalMax = 2.0f;
  l.yawRateMax = 30.0f;
  l.vectorHorizontal = false;
  l.requireFinite = false;
  return l;
}

/**
 * 宽包络: 闭环脚本飞行使用的限幅(合速度 4 m/s, 垂直 5 m/s)。
 * 水平按合速度等比缩放以保持航向, 且拒绝非有限值 —— 脚本飞行由
 * 位置误差乘以增益算出速度, 遥测异常时可能算出 NaN/Inf, 必须整条拒绝。
 */
inline VelocityLimits ScriptVelocityLimits() {
  VelocityLimits l;
  l.horizontalMax = 4.0f;
  l.verticalMax = 5.0f;
  l.yawRateMax = 30.0f;
  l.vectorHorizontal = true;
  l.requireFinite = true;
  return l;
}

}  // namespace maicro
