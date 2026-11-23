#include "VoiceFlightScript.h"

#include "FlightController.h"  // ?? FlightCmd ???

#include <algorithm>
#include <cmath>

#include "MaicroLogger.inl"

namespace maicro {
namespace {
using Stage = VoiceFlightStage;
using Clock = std::chrono::steady_clock;

// ==================== 业务参数(只属于语音脚本) ====================
// 这些常量原先散落在 FlightController 的实现里, 现全部收敛到本业务模块。
// 硬件层与 FlightController 都不认识它们。
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kDepartureLengthM = 20.0;
constexpr double kDepartureAngleRad = 30.0 * kPi / 180.0;
constexpr double kHeightToleranceM = 0.18;
constexpr double kPositionToleranceM = 0.55;
constexpr double kVerticalHoverRadiusM = 1.0;
// 出发上升高度: 原 20m 斜距@30° 等于上升 10m, 按要求降低 5m → 上升 5m,
// 水平距离仍沿用 20m*cos30°, 平飞段不变。
constexpr double kDepartureRiseM = 5.0;
// "各无人机准备"保持原有高度/速度: 自动起飞后再相对爬升 1m, 1.5m/s。
constexpr float kPrepareSpeedMps = 1.5f;
constexpr float kAscentSpeedMps = 5.0f;
constexpr float kDepartureSpeedMps = 4.0f;
constexpr double kDepartureFlatLengthM = 10.0;

// ?????: ???? FlightCmd ????, ?????? ??
// ????????????????(?? VOICE_PREPARE=11 ?? 12),
// ?"??"??"??"?????????????
constexpr int CMD_PREPARE = static_cast<int>(FlightCmd::VOICE_PREPARE);
constexpr int CMD_ASCEND = static_cast<int>(FlightCmd::VOICE_ASCEND);
constexpr int CMD_DEPART = static_cast<int>(FlightCmd::VOICE_DEPART);

const char *CmdLabel(int cmd) {
  switch (cmd) {
    case CMD_PREPARE: return "VOICE_PREPARE";
    case CMD_ASCEND: return "VOICE_ASCEND";
    case CMD_DEPART: return "VOICE_DEPART";
    default: return "VOICE_UNKNOWN";
  }
}

bool Moving(Stage stage) {
  return stage == Stage::PREPARING || stage == Stage::ASCENDING ||
         stage == Stage::DEPARTING;
}

bool PositionMode(const VehicleTelemetry &sample) {
  return sample.displayMode == MAICRO_DISPLAY_MODE_P_GPS ||
         sample.displayMode == MAICRO_DISPLAY_MODE_NAVI_SDK_CTRL;
}

bool IsWellFormed(const VehicleTelemetry &sample) {
  return std::isfinite(sample.altitude) && std::isfinite(sample.yawDeg) &&
         std::isfinite(sample.position.latitude) &&
         std::isfinite(sample.position.longitude);
}

void HorizontalOffset(const S_MaicroPosition &origin,
                      const S_MaicroPosition &position, double &north,
                      double &east) {
  const double radians = kPi / 180.0;
  north = (position.latitude - origin.latitude) * radians * kEarthRadiusM;
  const double deltaLon = std::remainder(position.longitude - origin.longitude, 360.0);
  east = deltaLon * radians * kEarthRadiusM * std::cos(origin.latitude * radians);
}
}  // namespace

VoiceFlightScript::VoiceFlightScript() {}
VoiceFlightScript::~VoiceFlightScript() {}

const char *VoiceFlightScript::StageName(Stage stage) {
  switch (stage) {
    case Stage::IDLE: return "IDLE";
    case Stage::PREPARING: return "PREPARING";
    case Stage::PREPARED: return "PREPARED";
    case Stage::ASCENDING: return "ASCENDING";
    case Stage::READY: return "READY";
    case Stage::DEPARTING: return "DEPARTING";
    case Stage::COMPLETE: return "COMPLETE";
    case Stage::FAILED: return "FAILED";
  }
  return "FAILED";
}

void VoiceFlightScript::Publish(const VoiceFlightStatus &status) {
  if (!eventSink_ || status.request.empty()) return;
  // Queue-only event sink: no SDK/network call from the flight worker.
  eventSink_(std::string("TRK,VOICE_EVT,VOICE2,") + StageName(status.stage) +
             "," + status.request + "," + status.detail + "\n");
}

void VoiceFlightScript::MarkInterrupted() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_.stage != Stage::IDLE && status_.stage != Stage::COMPLETE &&
      status_.stage != Stage::FAILED) {
    status_.stage = Stage::FAILED;
    status_.detail = "interrupted";
    command_ = 0;
    Publish(status_);
  }
}

VoiceFlightStatus VoiceFlightScript::Status() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_.stage == Stage::PREPARED || status_.stage == Stage::READY) {
    VehicleTelemetry sample{};
    if (ctx_->Vehicle()->ReadTelemetry(sample) &&
        (sample.landingCount != roundLandings_ ||
         sample.status != MAICRO_FLIGHT_STATUS_IN_AIR)) {
      status_.stage = Stage::FAILED;
      status_.detail = "landed";
    }
  }
  return status_;
}

const char *VoiceFlightScript::Request(int cmd, const std::string &request,
                                       VoiceFlightStatus &out, bool latest,
                                       uint64_t generation) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request == status_.request && !request.empty()) {
    if (cmd != command_) return "badrequest";
    out = status_;
    return nullptr;
  }
  if (latest && std::find(recentRequests_.begin(), recentRequests_.end(),
                          request) != recentRequests_.end()) return "superseded";
  if (!latest && Moving(status_.stage)) return "busy";
  if (!ctx_->Vehicle() || !ctx_->Vehicle()->IsFlightCtrlReady()) return "flightctrl";

  VehicleTelemetry sample{};
  if (!ctx_->Vehicle()->ReadTelemetry(sample) || !IsWellFormed(sample) ||
      !ctx_->IsPositionSane(sample.position)) return "telemetry";

  Stage next;
  switch (cmd) {
    case CMD_PREPARE:
      if (sample.status != MAICRO_FLIGHT_STATUS_STOPED) return "ground";
      next = Stage::PREPARING;
      break;
    case CMD_ASCEND:
      if (!latest && status_.stage == Stage::READY) return "already_ascended";
      if (!latest && status_.stage != Stage::PREPARED) return "need_prepare";
      next = Stage::ASCENDING;
      break;
    case CMD_DEPART:
      if (!latest && status_.stage == Stage::COMPLETE) return "already_departed";
      if (!latest && status_.stage != Stage::READY) return "need_ascend";
      next = Stage::DEPARTING;
      break;
    default:
      return "badcmd";
  }
  if (cmd != CMD_PREPARE) {
    if (sample.status != MAICRO_FLIGHT_STATUS_IN_AIR ||
        (!latest && sample.landingCount != roundLandings_)) return "landed";
    if (!PositionMode(sample)) return "mode";
  }

  // generation 由 FlightController 统一分配(按键 PostCmd 也递增同一个计数),
  // 本模块只使用不自增, 否则两套令牌会错位导致抢占失效。
  if (latest) {
    ctx_->Worker().Post(cmd, generation);
    recentRequests_.push_back(request);
    if (recentRequests_.size() > 64) recentRequests_.pop_front();
  } else if (!ctx_->Worker().PostIfIdle(cmd, generation)) {
    return "busy";
  }
  generation_ = generation;
  roundLandings_ = sample.landingCount;
  MAICRO_INFO("voice accepted id=%s command=%s generation=%llu latest=%d",
              request.c_str(), CmdLabel(cmd), (unsigned long long)generation, latest ? 1 : 0);
  command_ = cmd;
  status_ = {next, request, "-"};
  out = status_;
  Publish(out);
  return nullptr;
}

void VoiceFlightScript::Execute(int cmd) {
  std::string request;
  uint32_t landings;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ctx_->Worker().ExecutingToken() != generation_ ||
        command_ != cmd || !Moving(status_.stage)) return;
    request = status_.request;
    landings = roundLandings_;
  }

  MAICRO_INFO("voice executing id=%s command=%s", request.c_str(), CmdLabel(cmd));
  const char *failure = Run(cmd, landings);
  Stage complete = Stage::COMPLETE;
  if (cmd == CMD_PREPARE) complete = Stage::PREPARED;
  if (cmd == CMD_ASCEND) complete = Stage::READY;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request != status_.request || !Moving(status_.stage)) return;
    status_.stage = failure ? Stage::FAILED : complete;
    status_.detail = failure ? failure : "-";
    Publish(status_);
  }
  if (failure) {
    ctx_->Notify(std::string("语音脚本 ") + CmdLabel(cmd) + " 失败：" + failure);
  } else if (complete == Stage::PREPARED) {
    ctx_->Notify("语音准备完成，距起飞点高 1 m");
  } else if (complete == Stage::READY) {
    ctx_->Notify("本机升高完成，等待出发");
  } else {
    ctx_->Notify("语音出发完成，已到位悬停");
  }
}

const char *VoiceFlightScript::Run(int cmd, uint32_t landings) {
  VehicleBase *vehicle = ctx_->Vehicle();
  if (!vehicle->IsFlightCtrlReady()) return "flightctrl";
  if (ctx_->Worker().HasPending()) return "interrupted";

  VehicleTelemetry origin{};
  if (!vehicle->ReadTelemetry(origin) || !IsWellFormed(origin) ||
      !ctx_->IsPositionSane(origin.position)) return "telemetry";
  if (origin.landingCount != landings) return "landed";

  const auto controlLossCount = vehicle->ControlLossCount();
  bool acquired = false;
  const char *result = nullptr;
  if (cmd == CMD_PREPARE) {
    if (origin.status != MAICRO_FLIGHT_STATUS_STOPED) return "ground";
    if (!ctx_->Takeoff(&acquired, false)) {
      result = "takeoff";
    } else {
      // SDK 的起飞状态查询和遥测回调可能先后更新。
      const auto modeDeadline = Clock::now() + std::chrono::milliseconds(1200);
      for (;;) {
        VehicleTelemetry sample{};
        if (vehicle->ReadTelemetry(sample) &&
            sample.status == MAICRO_FLIGHT_STATUS_IN_AIR && PositionMode(sample)) break;
        if (Clock::now() >= modeDeadline) { result = "mode"; break; }
        if (!ctx_->Worker().SleepMs(50)) { result = "interrupted"; break; }
      }
    }
  } else {
    if (origin.status != MAICRO_FLIGHT_STATUS_IN_AIR) return "landed";
    if (!PositionMode(origin)) return "mode";
    acquired = vehicle->StartFlightControl() == 0;
    if (!acquired) result = "authority";
  }

  if (!result) {
    if (cmd == CMD_DEPART) {
      // Both legs use the heading and altitude origin captured for this command.
      const double yaw = origin.yawDeg * kPi / 180.0;
      const double horizontal = kDepartureLengthM * std::cos(kDepartureAngleRad);
      const double up = kDepartureRiseM;
      const double n = std::cos(yaw);
      const double e = std::sin(yaw);
      MAICRO_INFO("voice departure: rise=%.0fm flat=%.0fm speed=%.0fm/s", up,
                  kDepartureFlatLengthM, kDepartureSpeedMps);
      result = Move(origin, horizontal * n, horizontal * e, up,
                    kDepartureSpeedMps, controlLossCount, true);
      if (!result && ctx_->Worker().HasPending()) result = "interrupted";
      if (!result) {
        MAICRO_INFO("voice departure: entering level leg, same heading, height held");
        result = Move(origin, (horizontal + kDepartureFlatLengthM) * n,
                      (horizontal + kDepartureFlatLengthM) * e, up,
                      kDepartureSpeedMps, controlLossCount);
      }
    } else {
      // 准备/升高都只走 1m: 准备 1.5m/s 保持原样, 号码口令的升高用 5m/s 上限
      const float speed = cmd == CMD_ASCEND ? kAscentSpeedMps : kPrepareSpeedMps;
      result = Move(origin, 0.0, 0.0, 1.0, speed, controlLossCount);
    }
  }

  if (acquired && vehicle->ControlLossCount() == controlLossCount) {
    const bool released = vehicle->StopFlightControl() == 0;
    if (!released && !result) result = "release";
  }
  return result;
}

const char *VoiceFlightScript::Move(const VehicleTelemetry &origin,
                                    double north, double east, double up,
                                    float32_t speed, uint32_t controlLossCount,
                                    bool passThrough) {
  VehicleBase *vehicle = ctx_->Vehicle();
  if (speed <= 0.0f) return "speed";
  if (vehicle->ControlLossCount() != controlLossCount) return "authority";
  if (vehicle->SetVelocityControlGround()) return "mode";

  const bool verticalOnly = north == 0.0 && east == 0.0;
  const auto deadline = Clock::now() + std::chrono::seconds(
      15 + static_cast<int>(std::ceil(std::hypot(std::hypot(north, east), up) / speed)));
  Clock::time_point settled{};
  const char *failure = "timeout";
  bool sentMotion = false;
  while (Clock::now() < deadline) {
    if (ctx_->Worker().HasPending()) { failure = "interrupted"; break; }
    if (vehicle->ControlLossCount() != controlLossCount) { failure = "authority"; break; }
    VehicleTelemetry sample{};
    if (!vehicle->ReadTelemetry(sample) || !IsWellFormed(sample) ||
        !ctx_->IsPositionSane(sample.position)) { failure = "telemetry"; break; }
    if (sample.landingCount != origin.landingCount || sample.status != MAICRO_FLIGHT_STATUS_IN_AIR) {
      failure = "landed";
      break;
    }
    if (!PositionMode(sample)) { failure = "mode"; break; }

    double offsetN, offsetE;
    HorizontalOffset(origin.position, sample.position, offsetN, offsetE);
    const double errorN = north - offsetN;
    const double errorE = east - offsetE;
    const double errorUp = origin.altitude + up - sample.altitude;
    const double horizontalError = std::hypot(errorN, errorE);
    if (verticalOnly && horizontalError > kVerticalHoverRadiusM) {
      failure = "drift";
      break;
    }

    if ((verticalOnly || horizontalError <= kPositionToleranceM) &&
        std::abs(errorUp) <= kHeightToleranceM) {
      if (passThrough) return nullptr;  // No brake/dwell at the junction.
      if (settled == Clock::time_point{}) settled = Clock::now();
      if (vehicle->FeedVelocity(VelocityVector{}, ScriptVelocityLimits())) { failure = "joystick"; break; }
      if (Clock::now() - settled >= std::chrono::milliseconds(200)) {
        failure = nullptr;
        break;
      }
    } else {
      settled = {};
      const double length = verticalOnly ? std::abs(errorUp) : std::hypot(horizontalError, errorUp);
      // The old 0.9 proportional gain limited a 1m ascent to 0.9m/s
      // regardless of its speed setting. Use the requested cap for this
      // dedicated 1m ascent; reduce the command with remaining distance.
      // Overshoot correction remains slow, instead of commanding -5m/s.
      const bool fastAscent = verticalOnly && up > 0.0 && speed >= kAscentSpeedMps;
      const double gain = fastAscent && errorUp > 0.0 ? 5.0 : 0.9;
      const double cap = fastAscent && errorUp < 0.0 ? kPrepareSpeedMps : speed;
      const double scale = passThrough ? cap / length : std::min(gain, cap / length);
      if (!sentMotion) {
        MAICRO_INFO("voice first_velocity n=%.3f e=%.3f up=%.3f cap=%.1f pass=%d",
                    verticalOnly ? 0.0 : errorN * scale,
                    verticalOnly ? 0.0 : errorE * scale,
                    errorUp * scale, speed, passThrough ? 1 : 0);
      }
      // 显式 static_cast: 聚合初始化不允许 double->float 窄化,
      // 而误差与增益都是 double。原函数传参是隐式转换, 此处保持同值。
      if (vehicle->FeedVelocity(
              VelocityVector{
                  static_cast<float32_t>(verticalOnly ? 0.0 : errorN * scale),
                  static_cast<float32_t>(verticalOnly ? 0.0 : errorE * scale),
                  static_cast<float32_t>(errorUp * scale), 0.0f},
              ScriptVelocityLimits())) {
        failure = "joystick";
        break;
      }
      sentMotion = true;
    }
    if (!ctx_->Worker().SleepMs(50)) { failure = "interrupted"; break; }
  }
  if (failure && sentMotion && vehicle->ControlLossCount() == controlLossCount &&
      ctx_->Status() == MAICRO_FLIGHT_STATUS_IN_AIR) {
    const auto mode = ctx_->DisplayMode();
    if (mode == MAICRO_DISPLAY_MODE_P_GPS || mode == MAICRO_DISPLAY_MODE_NAVI_SDK_CTRL) {
      vehicle->FeedVelocity(VelocityVector{}, ScriptVelocityLimits());
    }
  }
  return failure;
}
}  // namespace maicro
