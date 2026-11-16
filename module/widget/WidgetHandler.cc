#include "WidgetHandler.h"

#include <mutex>

#include "MaicroLogger.inl"

namespace maicro {
WidgetHandler::WidgetHandler() {
  for (auto &v : widgetValue_) v = 0;
}

WidgetHandler::~WidgetHandler() {}

void WidgetHandler::ShowMessage(const std::string &msg) {
  static std::mutex s_mutex;
  std::lock_guard<std::mutex> lck(s_mutex);
  T_DjiReturnCode rc = DjiWidgetFloatingWindow_ShowMessage(msg.c_str());
  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_DEBUG("floating window show message error, 0x%08llX", rc);
  }
}

int64_t WidgetHandler::Init(const std::string &cfgDir,
                            GimbalController *gimbal,
                            FlightController *flight) {
  gimbal_ = gimbal;
  flight_ = flight;

  gimbal_->SetCmdDoneCallback([this](GimbalCmd cmd, int64_t ret) {
    if (ret != 0) {
      // 常见原因: 俯仰已到机械限位, 继续同方向转动会被拒绝
      ShowMessage(std::string("云台 ") + GimbalController::CmdName(cmd) +
                  " 失败, 可能已到限位");
    }
  });
  flight_->SetNotifier([](const std::string &msg) { ShowMessage(msg); });

  // userData 指向本对象, 回调为静态函数. 全部使用 button 类型:
  // Pilot 2 大屏面板为每行2列网格, list 控件会展开占多格导致排布错位
  static T_DjiWidgetHandlerListItem s_widgetList[] = {
      {WIDGET_INDEX_TAKEOFF, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_LANDING, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_PITCH_UP, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_PITCH_DOWN, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_RECENTER, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_FLY_TO_TARGET, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_MOVE_FORWARD, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_MOVE_BACKWARD, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_MOVE_LEFT, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_MOVE_RIGHT, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_GO_HOME, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
      {WIDGET_INDEX_GO_HOME_RTK, DJI_WIDGET_TYPE_BUTTON, SetWidgetValue,
       GetWidgetValue, nullptr},
  };
  for (auto &item : s_widgetList) item.userData = this;

  T_DjiReturnCode rc = DjiWidget_Init();
  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("widget init error, 0x%08llX", rc);
    return -1;
  }

  rc = DjiWidget_RegDefaultUiConfigByDirPath(cfgDir.c_str());
  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("widget register ui config error, dir: %s, 0x%08llX",
                 cfgDir.c_str(), rc);
    return -2;
  }

  rc = DjiWidget_RegHandlerList(
      s_widgetList, sizeof(s_widgetList) / sizeof(s_widgetList[0]));
  if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    MAICRO_ERROR("widget register handler list error, 0x%08llX", rc);
    return -3;
  }

  MAICRO_INFO("widget init ok, cfg dir: %s", cfgDir.c_str());
  MAICRO_INFO("  [0]TAKEOFF [1]LANDING [2]PITCH_UP [3]PITCH_DOWN "
              "[4]RECENTER [5]FLY_TO_TARGET [6]FWD [7]BACK [8]LEFT "
              "[9]RIGHT [10]GO_HOME [11]GO_HOME_RTK");
  return 0;
}

T_DjiReturnCode WidgetHandler::SetWidgetValue(E_DjiWidgetType type,
                                              uint32_t index, int32_t value,
                                              void *userData) {
  WidgetHandler *self = static_cast<WidgetHandler *>(userData);
  if (index < WIDGET_MAX_NUM) self->widgetValue_[index] = value;
  MAICRO_INFO("widget set value, type: %d, index: %u, value: %d", (int)type,
              index, value);

  // 只投递命令, 不在 SDK 回调线程里调用阻塞接口
  if (type != DJI_WIDGET_TYPE_BUTTON ||
      value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
  }

  switch (index) {
    case WIDGET_INDEX_TAKEOFF:
      self->flight_->PostCmd(FlightCmd::TAKEOFF);
      break;
    case WIDGET_INDEX_LANDING:
      self->flight_->PostCmd(FlightCmd::LANDING);
      break;
    case WIDGET_INDEX_PITCH_UP:
      self->gimbal_->PostCmd(GimbalCmd::PITCH_UP);
      break;
    case WIDGET_INDEX_PITCH_DOWN:
      self->gimbal_->PostCmd(GimbalCmd::PITCH_DOWN);
      break;
    case WIDGET_INDEX_RECENTER:
      self->gimbal_->PostCmd(GimbalCmd::RECENTER);
      break;
    case WIDGET_INDEX_FLY_TO_TARGET:
      self->flight_->PostCmd(FlightCmd::FLY_TO_TARGET);
      break;
    case WIDGET_INDEX_MOVE_FORWARD:
      self->flight_->PostCmd(FlightCmd::MOVE_FORWARD);
      break;
    case WIDGET_INDEX_MOVE_BACKWARD:
      self->flight_->PostCmd(FlightCmd::MOVE_BACKWARD);
      break;
    case WIDGET_INDEX_MOVE_LEFT:
      self->flight_->PostCmd(FlightCmd::MOVE_LEFT);
      break;
    case WIDGET_INDEX_MOVE_RIGHT:
      self->flight_->PostCmd(FlightCmd::MOVE_RIGHT);
      break;
    case WIDGET_INDEX_GO_HOME:
      self->flight_->PostCmd(FlightCmd::GO_HOME);
      break;
    case WIDGET_INDEX_GO_HOME_RTK:
      self->flight_->PostCmd(FlightCmd::GO_HOME_RTK);
      break;
    default:
      break;
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode WidgetHandler::GetWidgetValue(E_DjiWidgetType type,
                                              uint32_t index, int32_t *value,
                                              void *userData) {
  (void)type;
  WidgetHandler *self = static_cast<WidgetHandler *>(userData);
  *value = (index < WIDGET_MAX_NUM) ? self->widgetValue_[index].load() : 0;
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
}  // namespace maicro
