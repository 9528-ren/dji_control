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

 private:
  // **?????????, ???????**
  // unique_ptr ??????**??**????, ???"????"????,
  // ??????????????????? ?? ????? DeInit ?
  // ???????? delete ??:
  //   ???????(widget/mop ??) -> ??? flight/gimbal
  //   -> ?????????? track -> ?? vehicle/conf
  // ?????????, ?????????????, ????????
  //
  // ??????: mopCmd_ -> widget_ -> flight_ -> gimbal_ -> track_
  //              -> vehicle_ -> conf_
  std::unique_ptr<ConfM3Gimbal> conf_;
  std::unique_ptr<VehicleBase> vehicle_;
  std::unique_ptr<TrackStore> track_;
  std::unique_ptr<GimbalController> gimbal_;
  std::unique_ptr<FlightController> flight_;
  std::unique_ptr<WidgetHandler> widget_;
  std::unique_ptr<MopCommandServer> mopCmd_;
};
}  // namespace maicro
