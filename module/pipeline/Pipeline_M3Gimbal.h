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
  // **声明顺序即依赖顺序, 不可随意重排。**
  // unique_ptr 成员按声明的**逆序**自动析构, 因此把"被依赖者"放在前面,
  // 析构时就会先销毁依赖方、后销毁被依赖方 —— 这正是原先 DeInit 里
  // 靠注释维持的手工 delete 顺序:
  //   先切断命令来源(widget/mop 线程) -> 再释放 flight/gimbal
  //   -> 最后释放被它们引用的 track -> 最后 vehicle/conf
  // 改由类型系统保证后, 重排声明会直接改变析构顺序, 比注释约定可靠。
  //
  // 析构实际顺序: mopCmd_ -> widget_ -> flight_ -> gimbal_ -> track_
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
