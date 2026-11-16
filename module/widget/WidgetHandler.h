#pragma once
#include <atomic>
#include <string>

#include "controller/FlightController.h"
#include "controller/GimbalController.h"
#include "dji_widget.h"

namespace maicro {
// Pilot 2 负载控件面板(大屏为每行2列网格, 不使用会展开占格的 list 控件),
// 与 widget_cfg/widget_config.json 一致:
//   行1: index 0 起飞   | index 1 降落
//   行2: index 2 上仰   | index 3 下俯
//   行3: index 4 回中   | index 5 飞往目标
//   行4: index 6 前进   | index 7 后退
//   行5: index 8 左移   | index 9 右移
//   行6: index 10 返航  | index 11 RTK返航
// 执行结果显示在浮窗中.
class WidgetHandler {
 public:
  enum {
    WIDGET_INDEX_TAKEOFF = 0,
    WIDGET_INDEX_LANDING = 1,
    WIDGET_INDEX_PITCH_UP = 2,
    WIDGET_INDEX_PITCH_DOWN = 3,
    WIDGET_INDEX_RECENTER = 4,
    WIDGET_INDEX_FLY_TO_TARGET = 5,
    WIDGET_INDEX_MOVE_FORWARD = 6,
    WIDGET_INDEX_MOVE_BACKWARD = 7,
    WIDGET_INDEX_MOVE_LEFT = 8,
    WIDGET_INDEX_MOVE_RIGHT = 9,
    WIDGET_INDEX_GO_HOME = 10,
    WIDGET_INDEX_GO_HOME_RTK = 11,
    WIDGET_MAX_NUM = 16,
  };

  WidgetHandler();
  ~WidgetHandler();

  int64_t Init(const std::string &cfgDir, GimbalController *gimbal,
               FlightController *flight);

  // 浮窗显示消息(线程安全)
  static void ShowMessage(const std::string &msg);

 private:
  static T_DjiReturnCode SetWidgetValue(E_DjiWidgetType type, uint32_t index,
                                        int32_t value, void *userData);
  static T_DjiReturnCode GetWidgetValue(E_DjiWidgetType type, uint32_t index,
                                        int32_t *value, void *userData);

 private:
  GimbalController *gimbal_ = nullptr;
  FlightController *flight_ = nullptr;
  std::atomic<int32_t> widgetValue_[WIDGET_MAX_NUM];
};
}  // namespace maicro
