#include "GimbalController.h"

#include "MaicroLogger.inl"

namespace maicro {
GimbalController::GimbalController() {}

GimbalController::~GimbalController() { DeInit(); }

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

void GimbalController::DeInit(void) { worker_.Stop(); }

void GimbalController::PostCmd(GimbalCmd cmd) {
  int oldCmd = worker_.Post((int)cmd);
  if (oldCmd) {
    MAICRO_WARN("gimbal cmd %s not executed yet, replaced by %s",
                CmdName((GimbalCmd)oldCmd), CmdName(cmd));
  }
}

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
