#include "GimbalController.h"

#include "MaicroLogger.inl"

namespace maicro {
GimbalController::GimbalController() {}

GimbalController::~GimbalController() { DeInit(); }

/// 云台命令的可读名, 仅用于日志与协议回显。
const char *GimbalController::CmdName(GimbalCmd cmd) {
  switch (cmd) {
    case GimbalCmd::PITCH_UP:
      return "PITCH_UP";
    case GimbalCmd::PITCH_DOWN:
      return "PITCH_DOWN";
    case GimbalCmd::RECENTER:
      return "RECENTER";
    default:
      return "NONE";
  }
}

/**
 * 初始化云台控制器并启动执行线程。
 * mountPos 由 Pipeline 传入硬件层的探测结果(而非配置原值): 配置为 0
 * 表示自动探测, 原值对云台操作没有意义。
 * pitchStep 为每次按键转动的角度, rotateTime 为单次转动耗时。
 */
int64_t GimbalController::Init(VehicleBase *vehicle,
                               E_MaicroMountPosition mountPos, float pitchStep,
                               float rotateTime) {
  vehicle_ = vehicle;
  mountPos_ = mountPos;
  pitchStep_ = pitchStep;
  rotateTime_ = rotateTime;
  worker_.Start("gimbal", [this](int cmd) { Execute(cmd); });
  MAICRO_INFO("gimbal controller init ok, mount pos: %d, pitch step: %.1f, "
              "rotate time: %.1fs",
              (int)mountPos_, pitchStep_, rotateTime_);
  return 0;
}

/// 停止云台执行线程。析构会自动调用。
void GimbalController::DeInit(void) { worker_.Stop(); }

/**
 * 投递云台命令(线程安全, 非阻塞)。单槽覆盖语义同 TaskWorker::Post:
 * 连点"上仰"时后一次会取代尚未执行的前一次, 并打 WARN 提示被覆盖的命令。
 */
void GimbalController::PostCmd(GimbalCmd cmd) {
  int oldCmd = worker_.Post((int)cmd);
  if (oldCmd) {
    MAICRO_WARN("gimbal cmd %s not executed yet, replaced by %s",
                CmdName((GimbalCmd)oldCmd), CmdName(cmd));
  }
}

/**
 * 执行云台命令(运行在云台执行线程)。
 * 上仰/下俯按 pitchStep_ 做相对转动, 回中调用云台复位。
 * 完成后经 cmdDoneCb_ 回调结果, 供上层向 Pilot 浮窗提示成败。
 */
void GimbalController::Execute(int cmd) {
  int64_t ret = 0;
  switch ((GimbalCmd)cmd) {
    case GimbalCmd::PITCH_UP:
      ret = RotatePitch(+pitchStep_);
      break;
    case GimbalCmd::PITCH_DOWN:
      ret = RotatePitch(-pitchStep_);
      break;
    case GimbalCmd::RECENTER:
      ret = Recenter();
      break;
    default:
      return;
  }
  if (cmdDoneCb_) cmdDoneCb_((GimbalCmd)cmd, ret);
}

/**
 * 相对转动俯仰轴 dPitch 度, 耗时 rotateTime_ 秒。
 * **只用相对角模式**: 绝对角模式会把 roll/yaw 一并归零, 那会破坏
 * 飞手已调好的构图, 故此处固定 isRelative=true 且只给 pitch 分量。
 */
int64_t GimbalController::RotatePitch(float dPitch) {
  int64_t ret = vehicle_->RotateGimbal(mountPos_, dPitch, 0, 0, rotateTime_,
                                       true);
  if (ret == 0) {
    MAICRO_INFO("gimbal rotate pitch %+.1f deg ok", dPitch);
  } else {
    MAICRO_ERROR("gimbal rotate pitch %+.1f deg error, 0x%08llX", dPitch, ret);
  }
  return ret;
}

/// 云台回中(俯仰与航向复位到默认姿态)。
int64_t GimbalController::Recenter(void) {
  int64_t ret = vehicle_->ResetGimbal(mountPos_);
  if (ret == 0) {
    MAICRO_INFO("gimbal recenter ok");
  } else {
    MAICRO_ERROR("gimbal recenter error, 0x%08llX", ret);
  }
  return ret;
}
}  // namespace maicro
