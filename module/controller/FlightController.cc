#include "FlightController.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "MaicroLogger.inl"

namespace maicro {
// 离地高度低于该值时视为触发降落保护, 需确认降落(参考 PSDK 官方 sample)
#define LANDING_CONFIRM_HEIGHT 0.7f

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
    default:
      return "NONE";
  }
}

int64_t FlightController::Init(VehicleBase *vehicle, const S_FlightConf &conf) {
  vehicle_ = vehicle;
  conf_ = conf;
  worker_.Start("flight", [this](int cmd) { Execute(cmd); });
  MAICRO_INFO("flight controller init ok, target: lat %.8f, lon %.8f, "
              "height %.1f m, mea %.1f m, speed %u m/s, max distance %.0f m, "
              "auto confirm landing: %d",
              conf_.latitude, conf_.longitude, conf_.height,
              conf_.minEnrouteHeight, conf_.speed, conf_.maxDistance,
              conf_.autoConfirmLanding);
  return 0;
}

void FlightController::DeInit(void) { worker_.Stop(); }

void FlightController::PostCmd(FlightCmd cmd) {
  int oldCmd = worker_.Post((int)cmd);
  if (oldCmd) {
    MAICRO_WARN("flight cmd %s not executed yet, replaced by %s",
                CmdName((FlightCmd)oldCmd), CmdName(cmd));
  }
}

void FlightController::Execute(int cmd) {
  MAICRO_INFO("flight cmd %s start", CmdName((FlightCmd)cmd));
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
    default:
      return;
  }
  MAICRO_INFO("flight cmd %s end, result: %s", CmdName((FlightCmd)cmd),
              ok ? "ok" : "failed/interrupted");
}

bool FlightController::DoTakeoff(void) {
  if (GetStatus() == MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("飞机已在空中, 忽略起飞");
    return false;
  }

  // 在地面静止时记录起飞点位置(RTK融合坐标), 作为 RTK返航 按钮的目的地
  S_MaicroPosition pos;
  homePosValid_ = false;
  if (vehicle_->QueryDronePosition(pos) == 0 &&
      (pos.latitude != 0 || pos.longitude != 0)) {
    homePos_ = pos;
    homePosValid_ = true;
    uint8_t rtk = 0;
    vehicle_->QueryRtkSolution(rtk);
    MAICRO_INFO("home position recorded: lat %.8f, lon %.8f, alt %.2f, "
                "rtk solution %u",
                pos.latitude, pos.longitude, pos.altitude, rtk);
    if (rtk != 50) {
      Notify("注意: 起飞时RTK非固定解(%u), RTK返航精度受限", rtk);
    }
  } else {
    MAICRO_WARN("record home position failed, RTK go-home unavailable");
  }

  if (vehicle_->StartFlightControl()) {
    Notify("起飞失败: 获取控制权失败");
    return false;
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

  float32_t height = 0;
  vehicle_->QueryDroneRelativeHeight(height);
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

  vehicle_->StartFlightControl();
  if (vehicle_->FlightControlGoHome()) {
    Notify("返航失败: 飞控拒绝返航");
    return false;
  }
  if (!WaitDisplayMode(MAICRO_DISPLAY_MODE_NAVI_GO_HOME,
                       MAICRO_DISPLAY_MODE_AUTO_LANDING, 3000)) {
    Notify("返航失败: 未进入返航模式, 当前模式 %u", GetDisplayMode());
    return false;
  }
  Notify("返航中...");

  // 返航飞行阶段
  int cycles = 0;
  while (GetStatus() == MAICRO_FLIGHT_STATUS_IN_AIR &&
         GetDisplayMode() == MAICRO_DISPLAY_MODE_NAVI_GO_HOME) {
    if (!worker_.SleepMs(1000)) {
      MAICRO_WARN("go home monitor interrupted by new cmd");
      return false;
    }
    if (++cycles > 1800) {
      Notify("返航超时, 停止监控");
      return false;
    }
  }

  // 到达返航点后自动降落
  if (GetStatus() == MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("到达返航点, 降落中...");
    if (!WaitLanded(300)) return false;
  }
  vehicle_->StopFlightControl();
  Notify("返航完成, 已降落");
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

  // 机体坐标系: x 前为正, y 右为正. 参照 fangxian-M4 用摇杆速度闭环平移
  // 左右符号按实机飞行方向标定(真机验证: y 正为机头左侧)
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
      dirY = 1;
      dirName = "左移";
      break;
    case FlightCmd::MOVE_RIGHT:
      dirY = -1;
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

bool FlightController::DoGoHomeRtk(void) {
  if (GetStatus() != MAICRO_FLIGHT_STATUS_IN_AIR) {
    Notify("飞机不在空中, 忽略RTK返航");
    return false;
  }
  if (!homePosValid_) {
    Notify("RTK返航失败: 本次起飞未记录起飞点, 请先用起飞按钮起飞");
    return false;
  }

  // RTK 前提校验: 起飞点与当前位置都必须是固定解
  uint8_t sol = 0;
  if (!QueryRtkFixed(sol)) {
    Notify("RTK返航拒绝: RTK非固定解(状态 %u), 请检查RTK后重试", sol);
    return false;
  }

  S_MaicroPosition currPos;
  if (vehicle_->QueryDronePosition(currPos)) {
    Notify("RTK返航失败: 无法获取当前位置");
    return false;
  }
  double dist = HorizontalDistance(currPos.latitude, currPos.longitude,
                                   homePos_.latitude, homePos_.longitude);
  MAICRO_INFO("rtk go home: current lat %.8f lon %.8f, home lat %.8f "
              "lon %.8f, distance %.1f m",
              currPos.latitude, currPos.longitude, homePos_.latitude,
              homePos_.longitude, dist);

  // 距离过远保护: 超出配置的最大距离拒绝执行(与飞往目标同一限制)
  if (dist > conf_.maxDistance) {
    Notify("RTK返航拒绝: 距起飞点 %.0f m 超过限制 %.0f m", dist,
           conf_.maxDistance);
    return false;
  }

  // 起飞点高度也记录自RTK, 垂直方向按椭球高闭环
  const float32_t hTolerance = 1.0f;
  const double hTolerance2D = 1.0;
  bool ctrlFailed = false;
  bool arrived = false;
  Notify("RTK返航中, 距起飞点 %.1f m...", dist);

  if (vehicle_->StartFlightControl()) {
    Notify("RTK返航失败: 获取控制权失败");
    return false;
  }
  vehicle_->SetVelocityControlGround();

  // 地面坐标系(北正/东正)闭环飞回起飞点上方, 保持起飞点高度
  int cycles = 0;
  while (cycles++ < 3600) {  // 100ms x 3600 = 6min 超时
    if (vehicle_->QueryDronePosition(currPos)) break;
    if (!QueryRtkFixed(sol)) {
      // 飞行途中 RTK 掉固定解: 悬停等待 5s, 仍未恢复则中止
      bool recovered = false;
      for (int i = 0; i < 50; ++i) {
        vehicle_->FeedVelocityVector(0, 0, 0, 0);
        if (!worker_.SleepMs(100)) return false;
        if (QueryRtkFixed(sol)) {
          recovered = true;
          break;
        }
      }
      if (!recovered) {
        Notify("RTK返航中止: RTK失去固定解, 已悬停");
        for (int i = 0; i < 6; ++i) vehicle_->FeedVelocityVector(0, 0, 0, 0);
        return false;
      }
      continue;
    }

    const double R = 6378137.0;
    double dNorth = (homePos_.latitude - currPos.latitude) * R * M_PI / 180.0;
    double dEast = (homePos_.longitude - currPos.longitude) * R * M_PI / 180.0 *
                   cos(currPos.latitude * M_PI / 180.0);
    double dUp = homePos_.altitude - currPos.altitude;
    dist = sqrt(dNorth * dNorth + dEast * dEast);

    if (dist < hTolerance2D && fabs(dUp) < hTolerance) {
      arrived = true;
      break;
    }

    // 误差即速度指令(接近自动减速), FeedVelocityVector 内部限幅
    float32_t vN = dNorth > 3 ? 3 : (dNorth < -3 ? -3 : (float32_t)dNorth);
    float32_t vE = dEast > 3 ? 3 : (dEast < -3 ? -3 : (float32_t)dEast);
    float32_t vU = dUp > 2 ? 2 : (dUp < -2 ? -2 : (float32_t)dUp);
    if (dist < 2.0) {
      // 水平到位后只做垂直调整
      vN = 0;
      vE = 0;
    }
    if (vehicle_->FeedVelocityVector(vN, vE, vU, 0)) {
      ctrlFailed = true;
      MAICRO_ERROR("rtk go home: joystick command rejected");
      break;
    }
    if (cycles % 50 == 0) {
      Notify("RTK返航中, 剩余 %.1f m, 高度差 %.1f m", dist, dUp);
    }
    if (!worker_.SleepMs(100)) {
      MAICRO_WARN("rtk go home interrupted by new cmd");
      break;
    }
  }

  // 悬停稳定
  for (int i = 0; i < 10; ++i) {
    vehicle_->FeedVelocityVector(0, 0, 0, 0);
    worker_.SleepMs(50);
  }

  if (ctrlFailed) {
    Notify("RTK返航失败: 飞控拒绝摇杆指令");
    return false;
  }
  if (!arrived) {
    Notify("RTK返航未完成, 剩余 %.1f m", dist);
    return false;
  }

  // 回到起飞点上方, 走飞控自动降落
  Notify("已回到起飞点上方, 降落中...");
  if (vehicle_->FlightControlLanding()) {
    Notify("RTK返航: 降落指令被拒, 悬停在起飞点上方");
    return false;
  }
  if (!WaitLanded(300)) {
    vehicle_->StopFlightControl();
    return false;
  }
  vehicle_->StopFlightControl();
  Notify("RTK返航完成, 已在起飞点降落");
  return true;
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

void FlightController::Notify(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  MAICRO_INFO("[FLIGHT] %s", buf);
  if (notifier_) notifier_(buf);
}
}  // namespace maicro
