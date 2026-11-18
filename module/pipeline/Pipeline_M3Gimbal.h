#pragma once
#include <string>

#include "conf/ConfM3Gimbal.h"
#include "controller/FlightController.h"
#include "controller/GimbalController.h"
#include "controller/TrackStore.h"
#include "vehicle/VehicleDjiPsdk.h"
#include "widget/WidgetHandler.h"
#include "mop/MopCommandServer.h"

namespace maicro {
class PipelineM3Gimbal {
 public:
  explicit PipelineM3Gimbal(const std::string &cfgPath);
  ~PipelineM3Gimbal();

  /**********************************************************
   * 初始化函数
   **********************************************************/
  // pipeline初始化
  int64_t Init(void);

  // pipeline去初始化
  void DeInit(void);

  // 初始化配置
  int64_t InitConf(void);

  // 初始化飞机(PSDK核心/云台/飞控/数据订阅)
  int64_t InitVehicle(void);

  // 初始化云台控制
  int64_t InitGimbal(void);

  // 初始化飞行控制
  int64_t InitFlight(void);

  // 初始化控件
  int64_t InitWidget(void);

 public:
  std::string cfgPath_;
  ConfM3Gimbal *conf_ = nullptr;
  VehicleBase *vehicle_ = nullptr;
  GimbalController *gimbal_ = nullptr;
  FlightController *flight_ = nullptr;
  WidgetHandler *widget_ = nullptr;
  MopCommandServer *mopCmd_ = nullptr;
  // 航迹存储: 被 flight_ 与 mopCmd_ 引用, 故必须在两者之后释放(见 DeInit)
  TrackStore *track_ = nullptr;
};
}  // namespace maicro
