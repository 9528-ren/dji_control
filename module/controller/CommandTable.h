#pragma once
#include "controller/FlightController.h"
#include "controller/GimbalController.h"

namespace maicro {

/**
 * 指令目标(通用): 一条外部指令要投递给哪个执行器。
 */
enum class CmdTarget { FLIGHT, GIMBAL };

/**
 * 一条外部指令的完整定义(通用协议表)。
 *
 * **为什么需要这张表:** 原先同一套指令在三处各写一遍 ——
 * MopCommandServer 里 14 个 else-if 字符串比较、WidgetHandler 里 12 个
 * switch-case 按钮索引、App 侧再存一份名字。加一条指令要改三处,
 * 漏一处就是"按钮有反应但 MOP 回 ERROR:unknown"这类静默失效。
 *
 * 现在指令名、按钮索引、目标执行器、命令枚举值都集中在此表,
 * MOP 分发与控件回调都从表派生。加指令只需在表里加一行。
 *
 * widgetIndex 为 -1 表示该指令不出现在 Pilot 控件上(仅 MOP 可达)。
 */
struct CommandSpec {
  const char *name;      ///< MOP 协议里的指令名(与 App 约定, 不可随意改)
  int widgetIndex;       ///< Pilot 控件按钮索引; -1 = 无按钮
  CmdTarget target;      ///< 投递目标
  int code;              ///< FlightCmd 或 GimbalCmd 的枚举值(按 target 解释)
};

/**
 * 指令表。顺序即 Pilot 控件按钮顺序, 不可随意重排 ——
 * widgetIndex 与 widget_config.json 里的按钮位置一一对应。
 */
inline const CommandSpec *CommandTable(size_t &count) {
  static const CommandSpec kTable[] = {
      {"TAKEOFF",       0, CmdTarget::FLIGHT, (int)FlightCmd::TAKEOFF},
      {"LANDING",       1, CmdTarget::FLIGHT, (int)FlightCmd::LANDING},
      {"PITCH_UP",      2, CmdTarget::GIMBAL, (int)GimbalCmd::PITCH_UP},
      {"PITCH_DOWN",    3, CmdTarget::GIMBAL, (int)GimbalCmd::PITCH_DOWN},
      {"RECENTER",      4, CmdTarget::GIMBAL, (int)GimbalCmd::RECENTER},
      {"FLY_TO_TARGET", 5, CmdTarget::FLIGHT, (int)FlightCmd::FLY_TO_TARGET},
      {"MOVE_FORWARD",  6, CmdTarget::FLIGHT, (int)FlightCmd::MOVE_FORWARD},
      {"MOVE_BACKWARD", 7, CmdTarget::FLIGHT, (int)FlightCmd::MOVE_BACKWARD},
      {"MOVE_LEFT",     8, CmdTarget::FLIGHT, (int)FlightCmd::MOVE_LEFT},
      {"MOVE_RIGHT",    9, CmdTarget::FLIGHT, (int)FlightCmd::MOVE_RIGHT},
      {"GO_HOME",      10, CmdTarget::FLIGHT, (int)FlightCmd::GO_HOME},
      {"GO_HOME_RTK",  11, CmdTarget::FLIGHT, (int)FlightCmd::GO_HOME_RTK},
  };
  count = sizeof(kTable) / sizeof(kTable[0]);
  return kTable;
}

/// 按指令名查表; 未命中返回 nullptr
inline const CommandSpec *FindCommandByName(const std::string &name) {
  size_t n = 0;
  const CommandSpec *t = CommandTable(n);
  for (size_t i = 0; i < n; ++i) {
    if (name == t[i].name) return &t[i];
  }
  return nullptr;
}

/// 按控件按钮索引查表; 未命中返回 nullptr
inline const CommandSpec *FindCommandByWidgetIndex(int index) {
  size_t n = 0;
  const CommandSpec *t = CommandTable(n);
  for (size_t i = 0; i < n; ++i) {
    if (t[i].widgetIndex == index) return &t[i];
  }
  return nullptr;
}

// ==================== ???????? ====================
// ??????? WidgetHandler ?????, ???????????
// (???"??"??"??")?????????????????
static_assert((int)FlightCmd::TAKEOFF == 1, "FlightCmd order changed");
static_assert((int)FlightCmd::LANDING == 2, "FlightCmd order changed");
static_assert((int)GimbalCmd::PITCH_UP == 1, "GimbalCmd order changed");
static_assert((int)GimbalCmd::PITCH_DOWN == 2, "GimbalCmd order changed");
static_assert((int)GimbalCmd::RECENTER == 3, "GimbalCmd order changed");

}  // namespace maicro
