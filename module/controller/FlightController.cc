#include "FlightController.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "MaicroLogger.inl"

namespace maicro {
// 离地高度低于该值时视为触发降落保护, 需确认降落(参考 PSDK 官方 sample)
#define LANDING_CONFIRM_HEIGHT 0.7f

// 起飞目标高度: 飞控自动起飞只到约 1.2m, 且爬升速度由固件固定。
// 起飞后由本程序接手速度控制继续爬到该高度, 满足"起飞更快、停稳在 3m"的要求。
constexpr float kTakeoffHeightM = 3.0f;
constexpr float kTakeoffClimbSpeedMps = 4.0f;
constexpr float kTakeoffHeightTolM = 0.15f;

// 返航: 先原地把机头转向起飞点, 再平飞到位, 停稳 1s 后垂直降落。
// 到位判据比旧的 1m/3m 收紧, 否则"判到位→立刻降落"必然把 1~3m 的水平误差
// 直接变成落点偏差。
constexpr double kHomeArriveTolRtkM = 0.35;
constexpr double kHomeArriveTolGpsM = 1.2;
constexpr double kHomeApproachGain = 0.5;        // 速度 = 距离 x 0.5
constexpr double kHomeMinApproachSpeedMps = 0.3; // 末段最低速度, 避免停住不走
constexpr double kHomeMaxSpeedMps = 3.0;
constexpr int kHomeSettleCycles = 10;            // 100ms x 10 = 到位后稳定 1s
constexpr int kHomeYawCycles = 100;              // 100ms x 100 = 转向 10s 上限
constexpr float kHomeYawTolDeg = 5.0f;
constexpr float kHomeYawRateGain = 1.5f;
constexpr float kHomeYawRateMinDegps = 10.0f;    // 低于该角速率飞控可能不响应
constexpr float kHomeYawRateMaxDegps = 30.0f;

FlightController::FlightController() {}

FlightController::~FlightController() { DeInit(); }

const char *FlightController::CmdName(FlightCmd cmd) {
  switch (cmd) {
    case FlightCmd::TAKEOFF:
      return "TAKEOFF";
    case FlightCmd::LANDING:
      return "LANDING";
    case FlightCmd::FLY_TO_TARGET:
      return "FLY_TO_TARGET";
    case FlightCmd::GO_HOME:
      return "GO_HOME";
    case FlightCmd::MOVE_FORWARD:
      return "MOVE_FORWARD";
    case FlightCmd::MOVE_BACKWARD:
      return "MOVE_BACKWARD";
    case FlightCmd::MOVE_LEFT:
      return "MOVE_LEFT";
    case FlightCmd::MOVE_RIGHT:
      return "MOVE_RIGHT";
    case FlightCmd::GO_HOME_RTK:
      return "GO_HOME_RTK";
    case FlightCmd::TRACK_RUN:
      return "TRACK_RUN";
    case FlightCmd::VOICE_PREPARE:
      return "VOICE_PREPARE";
    case FlightCmd::VOICE_ASCEND:
      return "VOICE_ASCEND";
    case FlightCmd::VOICE_DEPART:
      return "VOICE_DEPART";
    default:
      return "NONE";
  }
}

int64_t FlightController::Init(VehicleBase *vehicle, const S_FlightConf &conf) {
  voiceScript_.Init(this);
  vehicle_ = vehicle;
  conf_ = conf;
  worker_.Start("flight", [this](int cmd) { Execute(cmd); });
  MAICRO_INFO("flight controller init ok, target: lat %.8f, lon %.8f, "
              "height %.1f m, mea %.1f m, speed %u m/s, max distance %.0f m, "
              "auto confirm landing: %d",
              conf_.latitude, conf_.longitude, conf_.height,
              conf_.minEnrouteHeight, conf_.speed, conf_.maxDistance,
              conf_.autoConfirmLanding);
  MAICRO_INFO("track conf: minSatellites=%u pointTimeout=%us tolerance=%.1fm "
              "gimbalSettle=%ums altCrossCheck=%.1fm requireNetRtk=%d "
              "stopOnRtkLoss=%d",
              conf_.trackMinSatellites, conf_.trackPointTimeoutS,
              conf_.trackArriveTolerance, conf_.trackGimbalSettleMs,
              conf_.trackAltCrossCheckM, conf_.trackRequireNetworkRtk ? 1 : 0,
              conf_.trackStopOnRtkLoss ? 1 : 0);
  return 0;
}

void FlightController::DeInit(void) { worker_.Stop(); }

void FlightController::PostCmd(FlightCmd cmd) {
  std::lock_guard<std::mutex> lock(voiceMutex_);
  ++voiceGeneration_;
  voiceScript_.MarkInterrupted();
  int oldCmd = worker_.Post((int)cmd, voiceGeneration_);
  if (oldCmd) {
    MAICRO_WARN("flight cmd %s not executed yet, replaced by %s",
                CmdName((FlightCmd)oldCmd), CmdName(cmd));
  }
}

void FlightController::Execute(int cmd) {
  {
    std::lock_guard<std::mutex> lock(voiceMutex_);
    if (worker_.ExecutingToken() != voiceGeneration_) return;
  }
  MAICRO_INFO("flight cmd %s start", CmdName((FlightCmd)cmd));
  const auto flightCmd = static_cast<FlightCmd>(cmd);
  if (flightCmd == FlightCmd::VOICE_PREPARE || flightCmd == FlightCmd::VOICE_ASCEND ||
      flightCmd == FlightCmd::VOICE_DEPART) {
    voiceScript_.Execute(static_cast<int>(flightCmd));
    return;
  }
  if (!vehicle_->IsFlightCtrlReady()) {
    Notify("飞控模块未初始化, 无法执行");
    return;
  }

  bool ok = false;
  switch ((FlightCmd)cmd) {
    case FlightCmd::TAKEOFF:
      ok = DoTakeoff();
      break;
    case FlightCmd::LANDING:
      ok = DoLanding();
      break;
    case FlightCmd::FLY_TO_TARGET:
      ok = DoFlyToTarget();
      break;
    case FlightCmd::GO_HOME:
      ok = DoGoHome();
      break;
    case FlightCmd::MOVE_FORWARD:
    case FlightCmd::MOVE_BACKWARD:
    case FlightCmd::MOVE_LEFT:
    case FlightCmd::MOVE_RIGHT:
      ok = DoMove((FlightCmd)cmd);
      break;
    case FlightCmd::GO_HOME_RTK:
      ok = DoGoHomeRtk();
      break;
    case FlightCmd::TRACK_RUN:
      ok = DoRunTrack();
      break;
    default:
      return;
  }
  MAICRO_INFO("flight cmd %s end, result: %s", CmdName((FlightCmd)cmd),
              ok ? "ok" : "failed/interrupted");
}

bool FlightController::DoTakeoff(bool *controlObtained, bool climbToTargetHeight) {
  if (GetStatus() == MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("飞机已在空中, 忽略起飞");
    return false;
  }

  // 在地面静止时记录起飞点位置(RTK融合坐标), 作为返航的目的地
  // **必须零初始化**: 下面失败分支会读 pos.satelliteNum 打日志, 而
  // QueryDronePosition 失败时不保证填充该结构体, 未初始化即读取属于未定义行为。
  S_MaicroPosition pos = {0};
  homePosValid_ = false;
  uint8_t rtk = 0;
  vehicle_->QueryRtkSolution(rtk);
  if (vehicle_->QueryDronePosition(pos) == 0 && IsHomePositionSane(pos)) {
    homePos_ = pos;
    homePosValid_ = true;
    MAICRO_INFO("home position recorded: lat %.8f, lon %.8f, alt %.2f, "
                "satellites %u, rtk solution %u",
                pos.latitude, pos.longitude, pos.altitude,
                (unsigned)pos.satelliteNum, (unsigned)rtk);
    if (rtk != 50) {
      // 按使用者要求返航不区分 RTK/GPS, 此处只如实告知精度等级, 不阻止返航
      Notify("注意: 起飞时非RTK固定解(%u), 返航按GPS精度执行, 落点误差可能数米",
             (unsigned)rtk);
    }
  } else {
    // **必须如实报告并明确后果**: 起飞点无效时返航只能改走飞控原生返航
    // (飞控自己的 Home 点), 精度与本程序的闭环返航不同。
    // 旧实现在此处只写一条 WARN 日志、界面上毫无提示, 使用者完全不知道
    // "起飞点根本没记下来", 直到按返航才发现行为异常。
    MAICRO_WARN("home position INVALID, closed-loop go-home unavailable: "
                "lat %.8f, lon %.8f, satellites %u, rtk %u",
                pos.latitude, pos.longitude,
                (unsigned)pos.satelliteNum, (unsigned)rtk);
    Notify("警告: 无有效定位(卫星 %u 颗), 未能记录起飞点。"
           "返航将改由飞控原生返航执行",
           (unsigned)pos.satelliteNum);
  }

  if (vehicle_->StartFlightControl()) {
    Notify("起飞失败: 获取控制权失败");
    return false;
  }
  if (controlObtained) {
    *controlObtained = true;
    if (worker_.HasPending()) return false;
  }
  if (vehicle_->FlightControlTakeoff()) {
    Notify("起飞失败: 飞控拒绝起飞");
    return false;
  }
  Notify("起飞中...");

  // 电机启动检查
  int cycles = 0;
  while (GetStatus() != MAICRO_FLIGHT_STATUS_ON_GROUND &&
         GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR &&
         GetDisplayMode() != MAICRO_DISPLAY_MODE_ENGINE_START) {
    if (++cycles > 20 || !worker_.SleepMs(100)) {
      Notify("起飞失败: 电机未启动");
      return false;
    }
  }

  // 离地检查
  cycles = 0;
  while (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    if (++cycles > 110 || !worker_.SleepMs(100)) {
      Notify("起飞失败: 电机已转但未离地");
      return false;
    }
  }

  // 等待自动起飞结束
  cycles = 0;
  while (GetDisplayMode() == MAICRO_DISPLAY_MODE_AUTO_TAKEOFF ||
         GetDisplayMode() == MAICRO_DISPLAY_MODE_ASSISTED_TAKEOFF) {
    if (++cycles > 60 || !worker_.SleepMs(1000)) {
      Notify("起飞监控中断, 当前模式 %u", GetDisplayMode());
      return false;
    }
  }

  // 续爬升: 自动起飞仅到约 1.2m, 这里用速度控制继续爬到 3m (2.5m/s),
  // 速度按剩余高度比例收敛, 到 0.15m 容差内松杆悬停。
  // 失败只影响这段续爬, 飞机仍在空中悬停, 不改变"起飞成功"的结论。
  float32_t height = 0;
  if (climbToTargetHeight && vehicle_->QueryDroneRelativeHeight(height) == 0 &&
      height < kTakeoffHeightM - kTakeoffHeightTolM) {
    MAICRO_INFO("takeoff climb: %.2f m -> %.2f m, cap %.1f m/s", height,
                kTakeoffHeightM, kTakeoffClimbSpeedMps);
    if (vehicle_->SetVelocityControlGround() == 0) {
      for (int i = 0; i < 300; ++i) {  // 100ms x 300 = 30s 上限
        if (vehicle_->QueryDroneRelativeHeight(height) != 0) break;
        const float32_t err = kTakeoffHeightM - height;
        if (fabsf(err) <= kTakeoffHeightTolM) break;
        float32_t vz = err * 3.0f;
        if (vz > kTakeoffClimbSpeedMps) vz = kTakeoffClimbSpeedMps;
        else if (vz < -kTakeoffClimbSpeedMps) vz = -kTakeoffClimbSpeedMps;
        if (vehicle_->FeedVelocity(VelocityVector{0, 0, vz, 0},
                                   ScriptVelocityLimits()) != 0) break;
        if (!worker_.SleepMs(100)) break;
      }
      vehicle_->FeedVelocity(VelocityVector{}, ScriptVelocityLimits());
      vehicle_->QueryDroneRelativeHeight(height);
    } else {
      MAICRO_WARN("takeoff climb skipped: velocity control unavailable");
    }
  }
  Notify("起飞完成, 相对高度 %.1f m", height);
  return true;
}

bool FlightController::DoLanding(void) {
  if (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("飞机不在空中, 忽略降落");
    return false;
  }

  vehicle_->StartFlightControl();
  if (vehicle_->FlightControlLanding()) {
    Notify("降落失败: 飞控拒绝降落");
    return false;
  }
  if (!WaitDisplayMode(MAICRO_DISPLAY_MODE_AUTO_LANDING,
                       MAICRO_DISPLAY_MODE_FORCE_AUTO_LANDING, 3000)) {
    Notify("降落失败: 未进入自动降落, 当前模式 %u", GetDisplayMode());
    return false;
  }
  Notify("降落中...");

  if (!WaitLanded(300)) return false;
  vehicle_->StopFlightControl();
  Notify("降落完成");
  return true;
}

bool FlightController::DoFlyToTarget(void) {
  if (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    // 地面下发会让飞机自动起飞, 这里要求先手动起飞, 行为更可控
    Notify("请先起飞, 再飞往目标点");
    return false;
  }
  if (conf_.latitude == 0 && conf_.longitude == 0) {
    Notify("未配置目标点, 请修改 cfg/m3_gimbal.cfg");
    return false;
  }

  S_MaicroPosition current;
  float32_t relHeight = 0;
  if (vehicle_->QueryDronePosition(current) ||
      vehicle_->QueryDroneRelativeHeight(relHeight) ||
      (current.latitude == 0 && current.longitude == 0)) {
    Notify("飞往目标点失败: 无法获取当前位置");
    return false;
  }

  double distance = HorizontalDistance(current.latitude, current.longitude,
                                       conf_.latitude, conf_.longitude);
  if (distance > conf_.maxDistance) {
    Notify("目标点距离 %.0f m 超过限制 %.0f m, 拒绝执行", distance,
           conf_.maxDistance);
    return false;
  }

  // 接口要求椭球高: 当前椭球高 + (目标相对高度 - 当前相对高度)
  S_MaicroPosition target;
  target.latitude = conf_.latitude;
  target.longitude = conf_.longitude;
  target.altitude = current.altitude + (conf_.height - relHeight);
  MAICRO_INFO("current: lat %.8f, lon %.8f, alt %.2f, rel height %.1f, "
              "satellites %u",
              current.latitude, current.longitude, current.altitude, relHeight,
              current.satelliteNum);

  if (vehicle_->FlyToPosition(target, conf_.minEnrouteHeight, conf_.speed)) {
    Notify("飞往目标点失败: 飞控拒绝任务");
    return false;
  }
  Notify("飞往目标点, 距离 %.0f m", distance);

  // 等待任务开始
  int cycles = 0;
  while (vehicle_->QueryFlyToMissionState() == 0) {
    if (++cycles > 50 || !worker_.SleepMs(100)) {
      Notify("任务已下发, 未收到任务状态, 停止监控");
      return false;
    }
  }

  // 等待任务结束
  cycles = 0;
  while (vehicle_->QueryFlyToMissionState() != 0) {
    if (!worker_.SleepMs(1000)) {
      MAICRO_WARN("fly to target monitor interrupted by new cmd");
      return false;
    }
    if (++cycles % 5 == 0) {
      Notify("飞往目标点, 剩余 %.0f m", vehicle_->QueryFlyToRemainDistance());
    }
    if (cycles > 1800) {
      Notify("飞往目标点超时, 停止监控");
      return false;
    }
  }

  vehicle_->QueryDronePosition(current);
  distance = HorizontalDistance(current.latitude, current.longitude,
                                conf_.latitude, conf_.longitude);
  Notify("已到达目标点, 偏差 %.1f m", distance);
  return true;
}

bool FlightController::DoGoHome(void) {
  if (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("飞机不在空中, 忽略返航");
    return false;
  }

  // ---- 起飞点不可用时退回飞控原生返航(而不是拒绝执行, 否则飞机会滞留在空中) ----
  if (!homePosValid_) {
    Notify("本次未记录起飞点, 改用飞控原生返航");
    return DoGoHomeNative();
  }
  S_MaicroPosition currPos;
  if (vehicle_->QueryDronePosition(currPos) != 0 ||
      (currPos.latitude == 0 && currPos.longitude == 0)) {
    Notify("无法获取当前位置, 改用飞控原生返航");
    return DoGoHomeNative();
  }

  // ---- 定位质量只决定"到位判据松紧", 不作为返航前置门槛 ----
  // 按使用者要求: 返航不区分 RTK 与 GPS, 两者都执行。
  // RTK 固定解可收敛到 1m 级; 仅 GPS 时存在数米漂移, 判据放宽到 3m,
  // 否则会因定位噪声永远判不到位而空转到超时。
  uint8_t sol = 0;
  const bool rtkFixed = QueryRtkFixed(sol);
  // 定位质量只决定"到位判据松紧": RTK 0.35m, 仅 GPS 1.2m。
  const double arriveTol2D = rtkFixed ? kHomeArriveTolRtkM : kHomeArriveTolGpsM;
  if (!rtkFixed) {
    Notify("RTK 非固定解(状态 %u), 按 GPS 精度返航, 落点误差可能数米", sol);
  }

  double dist = HorizontalDistance(currPos.latitude, currPos.longitude,
                                   homePos_.latitude, homePos_.longitude);
  MAICRO_INFO("go home: current lat %.8f lon %.8f alt %.2f, home lat %.8f "
              "lon %.8f alt %.2f, distance %.1f m, rtkFixed %d, tol %.1f m",
              currPos.latitude, currPos.longitude, currPos.altitude,
              homePos_.latitude, homePos_.longitude, homePos_.altitude, dist,
              rtkFixed ? 1 : 0, arriveTol2D);

  // 距离远超配置上限说明定位很可能已漂移, 闭环飞回去不安全 → 交给飞控原生返航
  // (飞控有自己的返航点与避障逻辑)。注意: 这里是**改道**而非拒绝执行。
  if (dist > conf_.maxDistance) {
    Notify("距起飞点 %.0f m 超过限制 %.0f m, 定位可能漂移, 改用飞控原生返航",
           dist, conf_.maxDistance);
    return DoGoHomeNative();
  }

  if (vehicle_->StartFlightControl()) {
    Notify("返航失败: 获取控制权失败");
    return false;
  }
  // 地面坐标系 NEU(x 北正, y 东正), 与下方 dNorth/dEast 的分量顺序配套
  vehicle_->SetVelocityControlGround();
  Notify("返航中, 距起飞点 %.1f m...", dist);

  bool ctrlFailed = false;
  bool arrived = false;
  bool interrupted = false;

  // ---- 阶段 1: 原地把机头转向起飞点(只转航向, 不平移) ----
  // 旧实现全程不下发航向指令, 飞机保持离场时的机头朝向横着/倒着飞回来,
  // 看起来就不是"沿原路返回"。这里先对准起飞点再走直线。
  if (dist > arriveTol2D) {
    const double R = 6378137.0;
    const double dNorth = (homePos_.latitude - currPos.latitude) * R * M_PI / 180.0;
    const double dEast = (homePos_.longitude - currPos.longitude) * R * M_PI / 180.0 *
                         cos(currPos.latitude * M_PI / 180.0);
    const double bearing = atan2(dEast, dNorth) * 180.0 / M_PI;  // 0=正北, 90=正东
    MAICRO_INFO("go home: bearing to home %.1f deg, yawing nose to home",
                bearing);
    for (int i = 0; i < kHomeYawCycles; ++i) {
      if (worker_.HasPending()) {
        interrupted = true;
        break;
      }
      float32_t yaw = 0;
      if (vehicle_->QueryDroneYaw(yaw)) break;
      double err = bearing - (double)yaw;
      while (err > 180.0) err -= 360.0;
      while (err < -180.0) err += 360.0;
      if (fabs(err) <= kHomeYawTolDeg) break;
      float32_t rate = (float32_t)(err * kHomeYawRateGain);
      if (rate > kHomeYawRateMaxDegps) rate = kHomeYawRateMaxDegps;
      else if (rate < -kHomeYawRateMaxDegps) rate = -kHomeYawRateMaxDegps;
      if (rate > 0.0f && rate < kHomeYawRateMinDegps) rate = kHomeYawRateMinDegps;
      else if (rate < 0.0f && rate > -kHomeYawRateMinDegps) rate = -kHomeYawRateMinDegps;
      if (vehicle_->FeedVelocityVector(0, 0, 0, rate)) {
        ctrlFailed = true;
        MAICRO_ERROR("go home: yaw command rejected");
        break;
      }
      if (!worker_.SleepMs(100)) {
        interrupted = true;
        break;
      }
    }
    vehicle_->FeedVelocityVector(0, 0, 0, 0);
  }

  // ---- 阶段 2: 平飞回起飞点上方, 速度随距离收敛 ----
  // 旧实现 5m 内把速度从 3m/s 阶跃到 1m/s, 冲过起飞点再刹停, 落点因此偏出 1m 以上。
  // 现在速度 = 距离 x 0.5 (0.3~3m/s), 接近时自然减速; 进入容差后还要稳定 1s。
  int settled = 0;
  int cycles = 0;
  const int maxCycles = 3600;  // 100ms x 3600 = 6min 上限
  while (!ctrlFailed && !interrupted && cycles++ < maxCycles) {
    if (vehicle_->QueryDronePosition(currPos)) break;

    const double R = 6378137.0;
    const double dNorth = (homePos_.latitude - currPos.latitude) * R * M_PI / 180.0;
    const double dEast = (homePos_.longitude - currPos.longitude) * R * M_PI / 180.0 *
                         cos(currPos.latitude * M_PI / 180.0);
    dist = sqrt(dNorth * dNorth + dEast * dEast);

    if (dist <= arriveTol2D) {
      vehicle_->FeedVelocityVector(0, 0, 0, 0);
      if (++settled >= kHomeSettleCycles) {
        arrived = true;
        break;
      }
      if (!worker_.SleepMs(100)) {
        interrupted = true;
        break;
      }
      continue;
    }
    settled = 0;

    double speed = dist * kHomeApproachGain;
    if (speed < kHomeMinApproachSpeedMps) speed = kHomeMinApproachSpeedMps;
    if (speed > kHomeMaxSpeedMps) speed = kHomeMaxSpeedMps;
    const float32_t vN = (float32_t)(dNorth / dist * speed);
    const float32_t vE = (float32_t)(dEast / dist * speed);
    // 垂直方向保持当前高度: 不在返航途中下降, 避免撞上起飞点与飞机之间的障碍;
    // 降落交给到位之后的 FlightControlLanding 垂直下降。
    if (vehicle_->FeedVelocityVector(vN, vE, 0, 0)) {
      ctrlFailed = true;
      MAICRO_ERROR("go home: joystick command rejected");
      break;
    }
    if (cycles % 50 == 0) {
      Notify("返航中, 剩余 %.1f m", dist);
    }
    if (!worker_.SleepMs(100)) {
      interrupted = true;
      MAICRO_WARN("go home interrupted by new cmd");
      break;
    }
  }

  // 无论何种退出路径, 先连发零速刹停, 否则飞机会带着最后一条速度指令继续飞
  for (int i = 0; i < 10; ++i) {
    vehicle_->FeedVelocityVector(0, 0, 0, 0);
    worker_.SleepMs(50);
  }

  if (interrupted) {
    vehicle_->StopFlightControl();
    Notify("返航已中止(收到新指令), 飞机悬停在当前位置");
    return false;
  }
  if (ctrlFailed) {
    vehicle_->StopFlightControl();
    Notify("返航失败: 飞控拒绝摇杆指令, 已悬停");
    return false;
  }
  if (!arrived) {
    // **不降落**: 没回到起飞点就降落 = 降落在未知地面, 比悬停危险得多
    vehicle_->StopFlightControl();
    Notify("返航未完成(超时或定位丢失), 距起飞点仍 %.1f m, 已悬停请手动处理",
           dist);
    return false;
  }

  // ---- 降落前复核确实已在起飞点上空 ----
  // 这是旧实现最致命的缺失: 它从不校验水平位置, 监控循环一退出就无条件宣告
  // "到达返航点, 降落中..."并原地降落(日志表现为"返航中"与"降落中"同一毫秒)。
  if (vehicle_->QueryDronePosition(currPos) == 0) {
    double dev = HorizontalDistance(currPos.latitude, currPos.longitude,
                                    homePos_.latitude, homePos_.longitude);
    if (dev > arriveTol2D * 2) {
      vehicle_->StopFlightControl();
      Notify("返航异常: 距起飞点仍 %.1f m, 已悬停不降落", dev);
      return false;
    }
    Notify("已回到起飞点上空(偏差 %.1f m), 降落中...", dev);
  } else {
    Notify("已回到起飞点上空, 降落中...");
  }

  if (vehicle_->FlightControlLanding()) {
    vehicle_->StopFlightControl();
    Notify("返航: 降落指令被拒, 已悬停在起飞点上方, 请手动降落");
    return false;
  }
  if (!WaitLanded(300)) {
    vehicle_->StopFlightControl();
    return false;
  }
  vehicle_->StopFlightControl();
  Notify("返航完成, 已在起飞点降落");
  return true;
}

bool FlightController::DoGoHomeNative(void) {
  if (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("飞机不在空中, 忽略返航");
    return false;
  }
  if (vehicle_->StartFlightControl()) {
    Notify("返航失败: 获取控制权失败");
    return false;
  }
  if (vehicle_->FlightControlGoHome()) {
    Notify("返航失败: 飞控拒绝返航");
    vehicle_->StopFlightControl();
    return false;
  }
  Notify("飞控原生返航中(自主返航并降落)...");

  // 只监控到落地为止。**绝不**依据显示模式推断"已到达返航点":
  // 旧实现用 WaitDisplayMode(GO_HOME, AUTO_LANDING) 把降落态也当作返航成功,
  // 且循环条件只认 GO_HOME 模式, 于是 0 次循环即退出并原地降落。
  int cycles = 0;
  while (GetStatus() == MAICRO_FLIGHT_STATUS_IN_AIR) {
    if (!worker_.SleepMs(1000)) {
      MAICRO_WARN("native go home monitor interrupted by new cmd");
      vehicle_->StopFlightControl();
      return false;
    }
    if (++cycles > 1800) {
      Notify("原生返航超时, 停止监控");
      vehicle_->StopFlightControl();
      return false;
    }
  }
  vehicle_->StopFlightControl();
  Notify("原生返航完成, 已降落");
  return true;
}

bool FlightController::DoMove(FlightCmd cmd) {
  if (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("请先起飞, 再使用方向按键");
    return false;
  }
  if (conf_.moveDistance <= 0) {
    Notify("平移距离未配置, 请检查 cfg move_distance");
    return false;
  }

  // 机体坐标系: PSDK 头文件 dji_flight_controller.h 明确定义
  //   DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE = 1
  //   "Set the x-y of body frame as the horizontal frame (FRU)"
  // FRU = Front-Right-Up, 即 **x 前为正, y 右为正, z 上为正**(航空标准机体系)。
  // 参照 fangxian-M4 用摇杆速度闭环平移。
  //
  // 修正记录(2026-11-17): 原实现 MOVE_LEFT 取 dirY=+1 / MOVE_RIGHT 取 -1,
  // 依据的是一条"真机验证 y 正为机头左侧"的标定注释 —— 该结论与 PSDK 的 FRU
  // 定义直接冲突, 且用户实测确认左右移反了, 故按 FRU 交换符号: 左移 y=-1, 右移 y=+1。
  // 注意: 此处符号必须与 SetVelocityControlBody() 设置的 BODY 坐标系配套;
  // 若将来改用地面坐标系(NEU), y 正为东, 符号含义完全不同, 不可沿用。
  float32_t dirX = 0, dirY = 0;
  const char *dirName = "前进";
  switch (cmd) {
    case FlightCmd::MOVE_FORWARD:
      dirX = 1;
      break;
    case FlightCmd::MOVE_BACKWARD:
      dirX = -1;
      dirName = "后退";
      break;
    case FlightCmd::MOVE_LEFT:
      dirY = -1;  // FRU: y 右为正, 故左移为负
      dirName = "左移";
      break;
    case FlightCmd::MOVE_RIGHT:
      dirY = 1;  // FRU: y 右为正
      dirName = "右移";
      break;
    default:
      return false;
  }

  S_MaicroPosition startPos, currPos;
  if (vehicle_->QueryDronePosition(startPos) ||
      (startPos.latitude == 0 && startPos.longitude == 0)) {
    Notify("平移失败: 无法获取当前位置");
    return false;
  }
  currPos = startPos;

  vehicle_->SetVelocityControlBody();
  MAICRO_INFO("move %s %.1f m: start lat %.8f, lon %.8f", dirName,
              conf_.moveDistance, startPos.latitude, startPos.longitude);
  Notify("%s %.0f m 中...", dirName, conf_.moveDistance);

  const float32_t tolerance = 0.5f;
  bool arrived = false;
  bool ctrlFailed = false;
  int cycles = 0;
  while (cycles++ < 600) {  // 50ms x 600 = 30s 超时
    double traveled = HorizontalDistance(startPos.latitude, startPos.longitude,
                                         currPos.latitude, currPos.longitude);
    float32_t disErr = conf_.moveDistance - (float32_t)traveled;
    if (disErr < tolerance) {
      arrived = true;
      break;
    }
    // 剩余距离作为速度指令, 内部限幅 3 m/s, 接近目标自动减速
    float32_t speed = disErr > 3.0f ? 3.0f : disErr;
    if (vehicle_->FeedVelocityVector(dirX * speed, dirY * speed, 0, 0)) {
      ctrlFailed = true;
      MAICRO_ERROR("move %s: joystick command rejected", dirName);
      break;
    }
    if (!worker_.SleepMs(50)) {
      MAICRO_WARN("move %s interrupted by new cmd", dirName);
      break;
    }
    if (vehicle_->QueryDronePosition(currPos)) break;
  }

  // 连发零速指令悬停刹停
  for (int i = 0; i < 6; ++i) {
    vehicle_->FeedVelocityVector(0, 0, 0, 0);
    worker_.SleepMs(50);
  }

  double finalDist = HorizontalDistance(startPos.latitude, startPos.longitude,
                                        currPos.latitude, currPos.longitude);
  MAICRO_INFO("move %s done: traveled %.2f m, arrived %d", dirName, finalDist,
              (int)arrived);
  if (ctrlFailed) {
    Notify("%s失败: 飞控拒绝摇杆指令", dirName);
    return false;
  }
  if (arrived) {
    Notify("%s完成, 实际 %.1f m", dirName, finalDist);
    return true;
  }
  Notify("%s结束, 实际 %.1f m", dirName, finalDist);
  return false;
}

bool FlightController::QueryRtkFixed(uint8_t &solution) {
  if (vehicle_->QueryRtkSolution(solution)) {
    return false;
  }
  // 50 = NARROW_INT, RTK 窄区固定解(厘米级), 视为 RTK 正常
  return solution == 50;
}

bool FlightController::IsHomePositionSane(const S_MaicroPosition &pos) {
  // 返航起飞点是"飞回去降落"的目标, 一旦记错就是让飞机飞向数千公里外的
  // 错误坐标或在不该降落的地方降落, 因此校验必须严格。
  //
  // 实测教训(2026-11-17 15:58): rtk solution=0(完全无定位)时 QueryDronePosition
  // 仍返回 lat 0.00000015, lon -0.00000037, alt -20.27 —— 这是"零点附近"的无效坐标,
  // 而旧判据 `latitude != 0 || longitude != 0` 因 0.00000015 不等于 0 而放行,
  // 导致起飞点被记成几内亚湾(0,0)附近的垃圾值。

  // 1) 经纬度必须远离零点: 无定位时融合位置会收敛到 (0,0) 附近的极小值。
  //    取 1e-4 度(约 11m)为下限, 真实起飞点不可能这么接近零点。
  if (fabs(pos.latitude) < 1e-4 || fabs(pos.longitude) < 1e-4) {
    return false;
  }
  // 2) 合法范围: 纬度 [-90,90], 经度 [-180,180], 越界说明是未初始化/错误数据
  if (pos.latitude < -90.0 || pos.latitude > 90.0) return false;
  if (pos.longitude < -180.0 || pos.longitude > 180.0) return false;
  // 3) 卫星数达标: 复用航迹配置的最小卫星数(默认 10), 卫星不足时定位不可信
  if (pos.satelliteNum < conf_.trackMinSatellites) {
    return false;
  }
  return true;
}

bool FlightController::DoGoHomeRtk(void) {
  // 按使用者要求: 返航**不区分 RTK 与 GPS**, 两个按钮行为完全一致 ——
  // 都是飞回起飞时记录的起飞点, 确认水平到位后再降落。
  // 故本函数只做委托, 不再保留独立的 RTK 专用实现:
  //   - 旧实现要求 QueryRtkFixed() 为真才肯执行, 非固定解直接拒绝返航,
  //     这会让仅有 GPS 定位时"返航"按钮完全不可用;
  //   - 旧实现的到位判据(dist<1.0)与减速区(dist<2.0 时水平速度清零)互相矛盾,
  //     飞机会停在 1~2m 之间既不移动也判不到位, 必然空转到 6 分钟超时。
  // 定位质量现在只在 DoGoHome 内部影响"到位判据松紧"(RTK 1m / GPS 3m),
  // 不再作为能否返航的门槛。
  return DoGoHome();
}

bool FlightController::WaitDisplayMode(uint8_t mode, uint8_t altMode,
                                       uint32_t timeoutMs) {
  uint32_t waited = 0;
  while (true) {
    uint8_t cur = GetDisplayMode();
    if (cur == mode || cur == altMode) return true;
    if (waited >= timeoutMs || !worker_.SleepMs(100)) return false;
    waited += 100;
  }
}

bool FlightController::WaitLanded(uint32_t timeoutS) {
  uint32_t lastConfirmS = 0;
  uint32_t waitedS = 0;
  bool protectNotified = false;

  while (GetStatus() == MAICRO_FLIGHT_STATUS_IN_AIR) {
    float32_t heightFusion = 100;
    vehicle_->QueryDroneHeightFusion(heightFusion);
    if (heightFusion <= LANDING_CONFIRM_HEIGHT) {
      if (conf_.autoConfirmLanding) {
        if (waitedS == 0 || waitedS - lastConfirmS >= 5) {
          MAICRO_INFO("height fusion %.2f m, confirm landing", heightFusion);
          vehicle_->FlightControlConfirmLanding();
          lastConfirmS = waitedS;
        }
      } else if (!protectNotified) {
        Notify("降落保护: 请在遥控器上确认降落");
        protectNotified = true;
      }
    }

    if (!worker_.SleepMs(1000)) {
      MAICRO_WARN("landing monitor interrupted by new cmd");
      return false;
    }
    if (++waitedS > timeoutS) {
      Notify("降落超时, 停止监控");
      return false;
    }
  }
  return true;
}

double FlightController::HorizontalDistance(double lat1, double lon1,
                                            double lat2, double lon2) {
  const double R = 6378137.0;
  double rlat1 = lat1 * M_PI / 180.0;
  double rlat2 = lat2 * M_PI / 180.0;
  double dlat = rlat2 - rlat1;
  double dlon = (lon2 - lon1) * M_PI / 180.0;
  double a = sin(dlat / 2) * sin(dlat / 2) +
             cos(rlat1) * cos(rlat2) * sin(dlon / 2) * sin(dlon / 2);
  return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

E_MaicroFlightStatus FlightController::GetStatus(void) {
  E_MaicroFlightStatus status = MAICRO_FLIGHT_STATUS_STOPED;
  vehicle_->QueryDroneStatus(status);
  return status;
}

uint8_t FlightController::GetDisplayMode(void) {
  uint8_t mode = 0;
  vehicle_->QueryDroneDisplayMode(mode);
  return mode;
}

void FlightController::Notify(const std::string &msg) {
  MAICRO_INFO("[FLIGHT] %s", msg.c_str());
  if (notifier_) notifier_(msg.c_str());
}

void FlightController::Notify(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  MAICRO_INFO("[FLIGHT] %s", buf);
  if (notifier_) notifier_(buf);
}

// ==================== 航迹功能 ====================

void FlightController::QueryRtkStatus(uint8_t &solution, bool &fixed,
                                      uint16_t &satellites,
                                      uint8_t &networkState) {
  solution = 0;
  fixed = false;
  satellites = 0;
  networkState = 0;

  fixed = QueryRtkFixed(solution);
  S_MaicroPosition pos;
  if (vehicle_->QueryDronePosition(pos) == 0) satellites = pos.satelliteNum;
  uint8_t ns = 0;
  if (vehicle_->QueryNetworkRtkState(ns) == 0) networkState = ns;
}

bool FlightController::WaitRtkStable(int times, uint32_t intervalMs,
                                     uint8_t &lastSol) {
  for (int i = 0; i < times; ++i) {
    if (!QueryRtkFixed(lastSol)) return false;
    S_MaicroPosition pos;
    if (vehicle_->QueryDronePosition(pos) != 0 ||
        pos.satelliteNum < conf_.trackMinSatellites) {
      MAICRO_WARN("rtk stable check: satellites %u < %u",
                  (unsigned)pos.satelliteNum, conf_.trackMinSatellites);
      return false;
    }
    if (i + 1 < times && !worker_.SleepMs(intervalMs)) return false;
  }
  return true;
}

int FlightController::RecordTrackPoint(S_TrackPoint &out,
                                       const char **errToken) {
  if (!track_) {
    if (errToken) *errToken = "nostore";
    return -1;
  }

  S_MaicroPosition pos;
  if (vehicle_->QueryDronePosition(pos) != 0 ||
      (fabs(pos.latitude) < 1e-6 && fabs(pos.longitude) < 1e-6)) {
    if (errToken) *errToken = "pos";
    return -2;
  }
  uint8_t sol = 0;
  bool fixed = QueryRtkFixed(sol);
  if (!fixed) {
    // 采点也要 RTK 门控: 非固定解采的坐标不准, 回放没有价值
    MAICRO_WARN("capture rejected: rtk solution %u != fixed", sol);
    if (errToken) *errToken = "rtk";
    return -3;
  }
  float32_t relH = 0;
  if (vehicle_->QueryDroneRelativeHeight(relH) != 0) {
    if (errToken) *errToken = "pos";
    return -4;
  }

  S_TrackPoint p;
  memset(&p, 0, sizeof(p));
  p.lat = pos.latitude;
  p.lon = pos.longitude;
  p.altEll = pos.altitude;
  p.relH = relH;
  p.satNum = pos.satelliteNum;
  p.rtkSol = sol;
  S_MaicroGimbalAngles ang;
  if (vehicle_->QueryGimbalAngles(ang) == 0) {
    p.gPitch = ang.pitch;
    p.gRoll = ang.roll;
    p.gYaw = ang.yaw;
  }
  float32_t dyaw = 0;
  if (vehicle_->QueryDroneYaw(dyaw) == 0) p.dYaw = dyaw;
  p.tsMs = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
               .count();

  int idx = track_->AddPoint(p);
  if (idx < 0) {
    // AddPoint 的负码: -1满 -2位置无效 -3非固定解 -4高度越界
    if (errToken) *errToken = (idx == -1) ? "full" : (idx == -4) ? "height" : "pos";
    return idx;
  }
  out = p;
  out.idx = (uint16_t)idx;
  MAICRO_INFO("track point captured #%d lat %.7f lon %.7f relH %.2f", idx,
              p.lat, p.lon, relH);
  return 0;
}

int FlightController::PreCheckTrackRun(const std::string &name, int reqStartIdx,
                                       S_TrackRunPlan &plan,
                                       const char **errToken) {
  if (!track_) {
    if (errToken) *errToken = "nostore";
    return -1;
  }
  if (IsTrackRunning()) {
    if (errToken) *errToken = "busy";
    return -2;
  }
  if (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    if (errToken) *errToken = "notinair";
    return -3;
  }

  S_Track t;
  int lr = track_->LoadTrack(name, t);
  if (lr == -2) {
    if (errToken) *errToken = "notfound";
    return -4;
  }
  if (lr != 0) {
    if (errToken) *errToken = "corrupt";
    return -5;
  }
  if (t.pts.empty()) {
    if (errToken) *errToken = "corrupt";
    return -6;
  }
  // 历史点必须是固定解采的, 否则坐标不可信, 拒绝执行整条航迹
  if (TrackStore::HasNonFixedRtkPoint(t)) {
    if (errToken) *errToken = "rtkpoint";
    return -7;
  }

  // RTK 单次门控(执行线程入口还会连续复检)
  uint8_t sol = 0;
  if (!QueryRtkFixed(sol)) {
    if (errToken) *errToken = "rtk";
    return -8;
  }
  if (conf_.trackRequireNetworkRtk) {
    uint8_t ns = 0;
    if (vehicle_->QueryNetworkRtkState(ns) != 0 || ns != 2) {
      if (errToken) *errToken = "rtk";
      return -9;
    }
  }

  S_MaicroPosition cur;
  if (vehicle_->QueryDronePosition(cur) != 0 ||
      (fabs(cur.latitude) < 1e-6 && fabs(cur.longitude) < 1e-6)) {
    if (errToken) *errToken = "pos";
    return -10;
  }

  // 求最近点
  int argmin = 0;
  double minDist = 1e18;
  for (size_t i = 0; i < t.pts.size(); ++i) {
    double d = HorizontalDistance(cur.latitude, cur.longitude, t.pts[i].lat,
                                  t.pts[i].lon);
    if (d < minDist) {
      minDist = d;
      argmin = (int)i;
    }
  }
  if (minDist > conf_.maxDistance) {
    MAICRO_WARN("nearest track point %.0f m > max %.0f m, refuse", minDist,
                conf_.maxDistance);
    if (errToken) *errToken = "toofar";
    return -11;
  }

  int start = (reqStartIdx > 0) ? reqStartIdx - 1 : argmin;
  if (start < 0 || start >= (int)t.pts.size()) {
    if (errToken) *errToken = "badidx";
    return -12;
  }

  plan.name = name;
  plan.startIdx = start + 1;  // 回显 1-based
  plan.total = (int)t.pts.size();
  plan.nearestDist = minDist;
  MAICRO_INFO("track precheck ok: '%s' total=%d start=%d nearest=%.1fm",
              name.c_str(), plan.total, plan.startIdx, minDist);
  return 0;
}

void FlightController::RequestTrackRun(const std::string &name, int startIdx) {
  {
    std::lock_guard<std::mutex> lk(trackMutex_);
    pendingName_ = name;
    pendingStartIdx_ = startIdx;
  }
  abortTrack_.store(false);
  PostCmd(FlightCmd::TRACK_RUN);
}

void FlightController::GetTrackStatus(S_TrackRunStatus &out) {
  std::lock_guard<std::mutex> lk(trackMutex_);
  out.state = (TrackState)trackState_.load();
  out.name = trackName_;
  out.curIdx = trackIdx_.load();
  out.total = trackTotal_.load();
  uint8_t sol = 0;
  QueryRtkFixed(sol);
  out.rtkSol = sol;
}

void FlightController::AbortTrack(const char *reason) {
  abortTrack_.store(true);
  MAICRO_WARN("track abort requested: %s", reason ? reason : "user");
  Notify("航迹中止请求: %s", reason ? reason : "用户");
}

void FlightController::PublishTrackEvent(const char *stateName) {
  if (!eventSink_) return;
  S_TrackRunStatus s;
  GetTrackStatus(s);
  char line[128];
  snprintf(line, sizeof(line), "TRK,EVT,%s,%d,%d,%s", stateName, s.curIdx,
           s.total, TrackStore::EncodeName(s.name).c_str());
  eventSink_(std::string(line) + "\n");
}

bool FlightController::DoRunTrack(void) {
  if (!track_) {
    Notify("航迹执行失败: 存储未就绪");
    return false;
  }

  std::string name;
  int reqStart;
  {
    std::lock_guard<std::mutex> lk(trackMutex_);
    name = pendingName_;
    reqStart = pendingStartIdx_;
    trackName_ = name;
  }

  S_Track t;
  if (track_->LoadTrack(name, t) != 0 || t.pts.empty()) {
    Notify("航迹执行失败: 加载 '%s' 失败", name.c_str());
    trackState_.store((int)TrackState::FAILED);
    PublishTrackEvent("ERR");
    return false;
  }

  // 入口复检(防预检与执行之间的状态变化, TOCTOU)
  if (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("航迹执行中止: 飞机不在空中");
    trackState_.store((int)TrackState::ABORTED);
    return false;
  }
  uint8_t sol = 0;
  if (!WaitRtkStable(5, 200, sol)) {
    Notify("航迹执行中止: RTK 未达稳定固定解(状态 %u)", sol);
    trackState_.store((int)TrackState::ABORTED);
    PublishTrackEvent("ABORT");
    return false;
  }
  if (TrackStore::HasNonFixedRtkPoint(t)) {
    Notify("航迹执行中止: 航迹含非固定解采集点");
    trackState_.store((int)TrackState::FAILED);
    PublishTrackEvent("ERR");
    return false;
  }

  // 重新求最近点(位置可能已变)
  S_MaicroPosition cur;
  if (vehicle_->QueryDronePosition(cur) != 0) {
    Notify("航迹执行中止: 无法获取当前位置");
    trackState_.store((int)TrackState::FAILED);
    return false;
  }
  int start = 0;
  if (reqStart > 0) {
    start = reqStart - 1;
  } else {
    double minDist = 1e18;
    for (size_t i = 0; i < t.pts.size(); ++i) {
      double d = HorizontalDistance(cur.latitude, cur.longitude, t.pts[i].lat,
                                    t.pts[i].lon);
      if (d < minDist) {
        minDist = d;
        start = (int)i;
      }
    }
    if (minDist > conf_.maxDistance) {
      Notify("航迹执行中止: 最近点 %.0f m 超限", minDist);
      trackState_.store((int)TrackState::ABORTED);
      PublishTrackEvent("ABORT");
      return false;
    }
  }
  if (start < 0 || start >= (int)t.pts.size()) {
    Notify("航迹执行中止: 起始点越界");
    trackState_.store((int)TrackState::FAILED);
    return false;
  }

  const int total = (int)t.pts.size();
  trackTotal_.store(total);
  trackState_.store((int)TrackState::RUNNING);
  trackRunning_.store(true);
  abortTrack_.store(false);
  Notify("航迹 '%s' 开始执行: 从第 %d 点起, 共 %d 点", name.c_str(), start + 1,
         total);
  PublishTrackEvent("RUN");

  bool ok = true;
  for (int k = start; k < total; ++k) {
    // 中止检查(每步): 用户 ABORT 或有新命令(降落/返航)抢占
    if (abortTrack_.load() || worker_.HasPending()) {
      Notify("航迹执行中止于第 %d 点(收到中止或新命令)", k + 1);
      trackState_.store((int)TrackState::ABORTED);
      trackIdx_.store(k);
      PublishTrackEvent("ABORT");
      trackRunning_.store(false);
      return false;
    }

    // 每点前复检 RTK(连续2次)
    if (!WaitRtkStable(2, 150, sol)) {
      // 掉解处理: 当前段目标点是固定解时采集的有效坐标, 让它飞完并在该点自动悬停,
      // 比用降级定位重新指定位置安全。这里停止下发后续航点。
      // 可选: trackStopOnRtkLoss 时用最后可信坐标强制停住(默认关)。
      Notify("RTK 失去固定解, 已停在第 %d 点, 请决定降落或返航", k + 1);
      trackState_.store((int)TrackState::ABORTED);
      trackIdx_.store(k);
      PublishTrackEvent("RTKLOSS");
      trackRunning_.store(false);
      return false;
    }

    const S_TrackPoint &wp = t.pts[k];
    S_MaicroPosition curPos;
    float32_t curRel = 0;
    if (vehicle_->QueryDronePosition(curPos) != 0 ||
        vehicle_->QueryDroneRelativeHeight(curRel) != 0) {
      Notify("航迹执行中止: 第 %d 点无法获取当前位置", k + 1);
      trackState_.store((int)TrackState::FAILED);
      trackRunning_.store(false);
      return false;
    }

    // 目标椭球高 = 当前椭球高 + (目标相对高 - 当前相对高), 与 DoFlyToTarget 一致
    S_MaicroPosition target;
    target.latitude = wp.lat;
    target.longitude = wp.lon;
    target.altitude = curPos.altitude + (wp.relH - curRel);
    target.satelliteNum = curPos.satelliteNum;

    // 交叉校验: 换算出的椭球高应与采集时的椭球高接近, 否则说明换了起飞点或基准漂移
    if (fabs(target.altitude - wp.altEll) > conf_.trackAltCrossCheckM) {
      Notify("航迹执行中止: 第 %d 点高度基准漂移 %.1f m(椭球高不匹配)", k + 1,
             fabs(target.altitude - wp.altEll));
      trackState_.store((int)TrackState::ABORTED);
      trackIdx_.store(k);
      PublishTrackEvent("ABORT");
      trackRunning_.store(false);
      return false;
    }

    double segDist = HorizontalDistance(curPos.latitude, curPos.longitude,
                                        wp.lat, wp.lon);
    if (segDist > conf_.maxDistance) {
      Notify("航迹执行中止: 第 %d 点段距 %.0f m 超限", k + 1, segDist);
      trackState_.store((int)TrackState::ABORTED);
      trackIdx_.store(k);
      PublishTrackEvent("ABORT");
      trackRunning_.store(false);
      return false;
    }
    float32_t mea = conf_.minEnrouteHeight > wp.relH ? conf_.minEnrouteHeight
                                                      : wp.relH;

    if (vehicle_->FlyToPosition(target, mea, conf_.speed)) {
      // 拒飞是系统性故障(不在空中/GPS无效/模式冲突), 跳过无意义, 中止整条
      Notify("航迹执行中止: 第 %d 点飞控拒绝任务", k + 1);
      trackState_.store((int)TrackState::FAILED);
      trackIdx_.store(k);
      PublishTrackEvent("ERR");
      trackRunning_.store(false);
      return false;
    }
    trackIdx_.store(k + 1);

    // 等任务开始
    int cycles = 0;
    while (vehicle_->QueryFlyToMissionState() == 0) {
      if (++cycles > 50 || !worker_.SleepMs(100)) break;
    }
    // 等任务结束, 期间复查 RTK 与卡死
    uint32_t timeout = conf_.trackPointTimeoutS;
    double estSec = (conf_.speed > 0) ? (segDist / conf_.speed) * 2 + 30 : 60;
    if (estSec > timeout) timeout = (uint32_t)estSec;
    float lastRemain = -1;
    int stallSec = 0;
    cycles = 0;
    bool segOk = false;
    while (worker_.SleepMs(200)) {
      uint8_t ms = vehicle_->QueryFlyToMissionState();
      float remain = vehicle_->QueryFlyToRemainDistance();
      if (ms == 0) {
        segOk = true;
        break;
      }
      // 卡死检测: 30s 内剩余距离无下降
      if (lastRemain >= 0 && remain >= lastRemain - 0.3f) {
        if (++stallSec > 150) {  // 150 * 200ms = 30s
          Notify("航迹执行中止: 第 %d 点飞行卡死(剩余 %.0f m 不下降)", k + 1,
                 remain);
          trackState_.store((int)TrackState::ABORTED);
          trackIdx_.store(k);
          PublishTrackEvent("ABORT");
          trackRunning_.store(false);
          return false;
        }
      } else {
        stallSec = 0;
      }
      lastRemain = remain;
      if (abortTrack_.load() || worker_.HasPending()) break;
      if ((uint32_t)cycles++ * 200 / 1000 > timeout) {
        Notify("航迹执行中止: 第 %d 点飞行超时", k + 1);
        trackState_.store((int)TrackState::ABORTED);
        trackIdx_.store(k);
        PublishTrackEvent("ABORT");
        trackRunning_.store(false);
        return false;
      }
      // 每 2s 推一次进度事件(兼作 App 侧活性信号)
      if (cycles % 10 == 0) PublishTrackEvent("RUN");
    }
    if (!segOk) {
      // 被中止/抢占, 交由循环顶部或此处收尾
      if (abortTrack_.load() || worker_.HasPending()) {
        Notify("航迹执行中止于第 %d 点", k + 1);
        trackState_.store((int)TrackState::ABORTED);
        trackIdx_.store(k);
        PublishTrackEvent("ABORT");
        trackRunning_.store(false);
        return false;
      }
    }

    // 云台回放: 只用相对角, 绝不触碰 roll/yaw(绝对模式会把它们归零)
    S_MaicroGimbalAngles now;
    if (vehicle_->QueryGimbalAngles(now) == 0) {
      float dPitch = wp.gPitch - now.pitch;
      if (dPitch > 90.0f) dPitch = 90.0f;
      if (dPitch < -90.0f) dPitch = -90.0f;
      // 转动耗时: 本类没有 GimbalController 的 rotateTime_, 由云台稳定上限派生,
      // 取 settle 的 1/3 并夹到 [0.3s, 2s], 给后续稳定等待留足余量
      float32_t rotateTime = conf_.trackGimbalSettleMs / 3000.0f;
      if (rotateTime > 2.0f) rotateTime = 2.0f;
      if (rotateTime < 0.3f) rotateTime = 0.3f;
      vehicle_->RotateGimbal(gimbalMountPos_, dPitch, 0, 0, rotateTime, true);
      uint32_t gw = 0;
      while (gw < conf_.trackGimbalSettleMs) {
        if (!worker_.SleepMs(200)) break;
        gw += 200;
        S_MaicroGimbalAngles chk;
        if (vehicle_->QueryGimbalAngles(chk) == 0 &&
            fabs(chk.pitch - wp.gPitch) < 1.5f)
          break;
      }
    }

    // 到位偏差
    S_MaicroPosition after;
    if (vehicle_->QueryDronePosition(after) == 0) {
      double dev = HorizontalDistance(after.latitude, after.longitude, wp.lat,
                                      wp.lon);
      if (dev > conf_.trackArriveTolerance * 3) {
        Notify("航迹执行中止: 第 %d 点偏差 %.1f m 过大", k + 1, dev);
        trackState_.store((int)TrackState::ABORTED);
        trackIdx_.store(k + 1);
        PublishTrackEvent("ABORT");
        trackRunning_.store(false);
        return false;
      }
      Notify("航迹 %s: %d/%d 完成, 偏差 %.1f m", name.c_str(), k + 1, total,
             dev);
    }
  }

  trackState_.store((int)TrackState::DONE);
  trackIdx_.store(total);
  Notify("航迹 '%s' 执行完成, 共 %d 点。飞机悬停在末点, 请手动降落或返航",
         name.c_str(), total);
  PublishTrackEvent("DONE");
  trackRunning_.store(false);
  return ok;
}

// ==================== 语音脚本转发(业务实现在 VoiceFlightScript) ====================

const char *FlightController::RequestVoiceFlight(FlightCmd cmd,
    const std::string &request, VoiceFlightStatus &out, bool latest) {
  // generation 必须由本类分配: 按键 PostCmd 也递增同一个计数,
  // 两边共用才能正确抢占。业务模块只使用不自增。
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(voiceMutex_);
    generation = voiceGeneration_ + 1;
  }
  const char *err = voiceScript_.Request(static_cast<int>(cmd), request, out,
                                         latest, generation);
  if (!err) {
    std::lock_guard<std::mutex> lock(voiceMutex_);
    voiceGeneration_ = generation;
  }
  return err;
}

VoiceFlightStatus FlightController::GetVoiceFlightStatus() {
  return voiceScript_.Status();
}

const char *FlightController::VoiceStageName(VoiceFlightStage stage) {
  return VoiceFlightScript::StageName(stage);
}

}  // namespace maicro
