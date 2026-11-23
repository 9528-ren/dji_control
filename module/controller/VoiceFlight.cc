#include "FlightController.h"

#include <algorithm>
#include <cmath>

#include "MaicroLogger.inl"

namespace maicro {
namespace {
using Stage = VoiceFlightStage;
using Clock = std::chrono::steady_clock;

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

bool Moving(Stage stage) {
  return stage == Stage::PREPARING || stage == Stage::ASCENDING ||
         stage == Stage::DEPARTING;
}

bool PositionMode(const VoiceFlightTelemetry &sample) {
  return sample.displayMode == MAICRO_DISPLAY_MODE_P_GPS ||
         sample.displayMode == MAICRO_DISPLAY_MODE_NAVI_SDK_CTRL;
}

bool IsWellFormed(const VoiceFlightTelemetry &sample) {
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

const char *FlightController::VoiceStageName(Stage stage) {
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

void FlightController::PublishVoiceStatus(const VoiceFlightStatus &status) {
  if (!eventSink_ || status.request.empty()) return;
  // Queue-only event sink: no SDK/network call from the flight worker.
  eventSink_(std::string("TRK,VOICE_EVT,VOICE2,") + VoiceStageName(status.stage) +
             "," + status.request + "," + status.detail + "\n");
}

VoiceFlightStatus FlightController::GetVoiceFlightStatus() {
  std::lock_guard<std::mutex> lock(voiceMutex_);
  if (voiceStatus_.stage == Stage::PREPARED || voiceStatus_.stage == Stage::READY) {
    VoiceFlightTelemetry sample{};
    if (vehicle_->ReadVoiceFlightTelemetry(sample) &&
        (sample.landings != voiceRoundLandings_ ||
         sample.status != MAICRO_FLIGHT_STATUS_IN_AIR)) {
      voiceStatus_.stage = Stage::FAILED;
      voiceStatus_.detail = "landed";
    }
  }
  return voiceStatus_;
}

const char *FlightController::RequestVoiceFlight(FlightCmd cmd,
    const std::string &request, VoiceFlightStatus &out, bool latest) {
  std::lock_guard<std::mutex> lock(voiceMutex_);
  if (request == voiceStatus_.request && !request.empty()) {
    if (cmd != voiceCommand_) return "badrequest";
    out = voiceStatus_;
    return nullptr;
  }
  if (latest && std::find(voiceRecentRequests_.begin(), voiceRecentRequests_.end(),
                          request) != voiceRecentRequests_.end()) return "superseded";
  if (!latest && Moving(voiceStatus_.stage)) return "busy";
  if (!vehicle_ || !vehicle_->IsFlightCtrlReady()) return "flightctrl";

  VoiceFlightTelemetry sample{};
  if (!vehicle_->ReadVoiceFlightTelemetry(sample) || !IsWellFormed(sample) ||
      !IsHomePositionSane(sample.position)) return "telemetry";

  Stage next;
  switch (cmd) {
    case FlightCmd::VOICE_PREPARE:
      if (sample.status != MAICRO_FLIGHT_STATUS_STOPED) return "ground";
      next = Stage::PREPARING;
      break;
    case FlightCmd::VOICE_ASCEND:
      if (!latest && voiceStatus_.stage == Stage::READY) return "already_ascended";
      if (!latest && voiceStatus_.stage != Stage::PREPARED) return "need_prepare";
      next = Stage::ASCENDING;
      break;
    case FlightCmd::VOICE_DEPART:
      if (!latest && voiceStatus_.stage == Stage::COMPLETE) return "already_departed";
      if (!latest && voiceStatus_.stage != Stage::READY) return "need_ascend";
      next = Stage::DEPARTING;
      break;
    default:
      return "badcmd";
  }
  if (cmd != FlightCmd::VOICE_PREPARE) {
    if (sample.status != MAICRO_FLIGHT_STATUS_IN_AIR ||
        (!latest && sample.landings != voiceRoundLandings_)) return "landed";
    if (!PositionMode(sample)) return "mode";
  }

  const auto generation = voiceGeneration_ + 1;
  if (latest) {
    worker_.Post(static_cast<int>(cmd), generation);
    voiceRecentRequests_.push_back(request);
    if (voiceRecentRequests_.size() > 64) voiceRecentRequests_.pop_front();
  } else if (!worker_.PostIfIdle(static_cast<int>(cmd), generation)) {
    return "busy";
  }
  voiceGeneration_ = generation;
  voiceRoundLandings_ = sample.landings;
  MAICRO_INFO("voice accepted id=%s command=%s generation=%llu latest=%d",
              request.c_str(), CmdName(cmd), (unsigned long long)generation, latest ? 1 : 0);
  voiceCommand_ = cmd;
  voiceStatus_ = {next, request, "-"};
  out = voiceStatus_;
  PublishVoiceStatus(out);
  return nullptr;
}

void FlightController::ExecuteVoiceFlight(FlightCmd cmd) {
  std::string request;
  uint32_t landings;
  {
    std::lock_guard<std::mutex> lock(voiceMutex_);
    if (worker_.ExecutingToken() != voiceGeneration_ ||
        voiceCommand_ != cmd || !Moving(voiceStatus_.stage)) return;
    request = voiceStatus_.request;
    landings = voiceRoundLandings_;
  }

  MAICRO_INFO("voice executing id=%s command=%s", request.c_str(), CmdName(cmd));
  const char *failure = DoVoiceFlight(cmd, landings);
  Stage complete = Stage::COMPLETE;
  if (cmd == FlightCmd::VOICE_PREPARE) complete = Stage::PREPARED;
  if (cmd == FlightCmd::VOICE_ASCEND) complete = Stage::READY;
  {
    std::lock_guard<std::mutex> lock(voiceMutex_);
    if (request != voiceStatus_.request || !Moving(voiceStatus_.stage)) return;
    voiceStatus_.stage = failure ? Stage::FAILED : complete;
    voiceStatus_.detail = failure ? failure : "-";
    PublishVoiceStatus(voiceStatus_);
  }
  if (failure) {
    Notify("语音脚本 %s 失败：%s", CmdName(cmd), failure);
  } else if (complete == Stage::PREPARED) {
    Notify("语音准备完成，距起飞点高 1 m");
  } else if (complete == Stage::READY) {
    Notify("本机升高完成，等待出发");
  } else {
    Notify("语音出发完成，已到位悬停");
  }
}

const char *FlightController::DoVoiceFlight(FlightCmd cmd, uint32_t landings) {
  if (!vehicle_->IsFlightCtrlReady()) return "flightctrl";
  if (worker_.HasPending()) return "interrupted";

  VoiceFlightTelemetry origin{};
  if (!vehicle_->ReadVoiceFlightTelemetry(origin) || !IsWellFormed(origin) ||
      !IsHomePositionSane(origin.position)) return "telemetry";
  if (origin.landings != landings) return "landed";

  const auto controlLossCount = vehicle_->VoiceControlLossCount();
  bool acquired = false;
  const char *result = nullptr;
  if (cmd == FlightCmd::VOICE_PREPARE) {
    if (origin.status != MAICRO_FLIGHT_STATUS_STOPED) return "ground";
    if (!DoTakeoff(&acquired, false)) {
      result = "takeoff";
    } else {
      // SDK 的起飞状态查询和遥测回调可能先后更新。
      const auto modeDeadline = Clock::now() + std::chrono::milliseconds(1200);
      for (;;) {
        VoiceFlightTelemetry sample{};
        if (vehicle_->ReadVoiceFlightTelemetry(sample) &&
            sample.status == MAICRO_FLIGHT_STATUS_IN_AIR && PositionMode(sample)) break;
        if (Clock::now() >= modeDeadline) { result = "mode"; break; }
        if (!worker_.SleepMs(50)) { result = "interrupted"; break; }
      }
    }
  } else {
    if (origin.status != MAICRO_FLIGHT_STATUS_IN_AIR) return "landed";
    if (!PositionMode(origin)) return "mode";
    acquired = vehicle_->StartFlightControl() == 0;
    if (!acquired) result = "authority";
  }

  if (!result) {
    if (cmd == FlightCmd::VOICE_DEPART) {
      // Both legs use the heading and altitude origin captured for this command.
      const double yaw = origin.yawDeg * kPi / 180.0;
      const double horizontal = kDepartureLengthM * std::cos(kDepartureAngleRad);
      const double up = kDepartureRiseM;
      const double n = std::cos(yaw);
      const double e = std::sin(yaw);
      MAICRO_INFO("voice departure: rise=%.0fm flat=%.0fm speed=%.0fm/s", up,
                  kDepartureFlatLengthM, kDepartureSpeedMps);
      result = MoveVoiceFlight(origin, horizontal * n, horizontal * e, up,
                               kDepartureSpeedMps, controlLossCount, true);
      if (!result && worker_.HasPending()) result = "interrupted";
      if (!result) {
        MAICRO_INFO("voice departure: entering level leg, same heading, height held");
        result = MoveVoiceFlight(origin, (horizontal + kDepartureFlatLengthM) * n,
                                 (horizontal + kDepartureFlatLengthM) * e, up,
                                 kDepartureSpeedMps, controlLossCount);
      }
    } else {
      // 准备/升高都只走 1m: 准备 1.5m/s 保持原样, 号码口令的升高用 5m/s 上限
      const float speed = cmd == FlightCmd::VOICE_ASCEND
          ? kAscentSpeedMps : kPrepareSpeedMps;
      result = MoveVoiceFlight(origin, 0.0, 0.0, 1.0, speed, controlLossCount);
    }
  }

  if (acquired && vehicle_->VoiceControlLossCount() == controlLossCount) {
    const bool released = vehicle_->StopFlightControl() == 0;
    if (!released && !result) result = "release";
  }
  return result;
}

const char *FlightController::MoveVoiceFlight(const VoiceFlightTelemetry &origin,
    double north, double east, double up, float32_t speed, uint32_t controlLossCount,
    bool passThrough) {
  if (speed <= 0.0f) return "speed";
  if (vehicle_->VoiceControlLossCount() != controlLossCount) return "authority";
  if (vehicle_->SetVelocityControlGround()) return "mode";

  const bool verticalOnly = north == 0.0 && east == 0.0;
  const auto deadline = Clock::now() + std::chrono::seconds(
      15 + static_cast<int>(std::ceil(std::hypot(std::hypot(north, east), up) / speed)));
  Clock::time_point settled{};
  const char *failure = "timeout";
  bool sentMotion = false;
  while (Clock::now() < deadline) {
    if (worker_.HasPending()) { failure = "interrupted"; break; }
    if (vehicle_->VoiceControlLossCount() != controlLossCount) { failure = "authority"; break; }
    VoiceFlightTelemetry sample{};
    if (!vehicle_->ReadVoiceFlightTelemetry(sample) || !IsWellFormed(sample) ||
        !IsHomePositionSane(sample.position)) { failure = "telemetry"; break; }
    if (sample.landings != origin.landings || sample.status != MAICRO_FLIGHT_STATUS_IN_AIR) {
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
      if (vehicle_->FeedVoiceFlightVelocityVector(0, 0, 0, 0)) { failure = "joystick"; break; }
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
      if (vehicle_->FeedVoiceFlightVelocityVector(
              verticalOnly ? 0.0f : errorN * scale,
              verticalOnly ? 0.0f : errorE * scale, errorUp * scale, 0.0f)) {
        failure = "joystick";
        break;
      }
      sentMotion = true;
    }
    if (!worker_.SleepMs(50)) { failure = "interrupted"; break; }
  }
  if (failure && sentMotion && vehicle_->VoiceControlLossCount() == controlLossCount &&
      GetStatus() == MAICRO_FLIGHT_STATUS_IN_AIR) {
    const auto mode = GetDisplayMode();
    if (mode == MAICRO_DISPLAY_MODE_P_GPS || mode == MAICRO_DISPLAY_MODE_NAVI_SDK_CTRL) {
      vehicle_->FeedVoiceFlightVelocityVector(0, 0, 0, 0);
    }
  }
  return failure;
}
}  // namespace maicro
