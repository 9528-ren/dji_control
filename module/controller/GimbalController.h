#pragma once
#include "TaskWorker.h"
#include "vehicle/VehicleBase.h"

namespace maicro {
enum class GimbalCmd { NONE = 0, PITCH_UP, PITCH_DOWN, RECENTER };

// 云台控制: 命令由工作线程执行
class GimbalController {
 public:
  typedef std::function<void(GimbalCmd cmd, int64_t ret)> CmdDoneCallback;

  GimbalController();
  ~GimbalController();

  int64_t Init(VehicleBase *vehicle, E_MaicroMountPosition mountPos,
               float pitchStep, float rotateTime);
  void DeInit(void);

  // 投递命令(线程安全, 非阻塞)
  void PostCmd(GimbalCmd cmd);

  void SetCmdDoneCallback(CmdDoneCallback cb) { cmdDoneCb_ = cb; }

  static const char *CmdName(GimbalCmd cmd);

 private:
  void Execute(int cmd);
  int64_t RotatePitch(float dPitch);
  int64_t Recenter(void);

 private:
  VehicleBase *vehicle_ = nullptr;
  E_MaicroMountPosition mountPos_ = MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1;
  float pitchStep_ = 10.0f;
  float rotateTime_ = 1.0f;
  TaskWorker worker_;
  CmdDoneCallback cmdDoneCb_ = nullptr;
};
}  // namespace maicro
