#pragma once
#include <string>

#include "TaskWorker.h"
#include "vehicle/VehicleBase.h"

namespace maicro {
enum class FlightCmd {
  NONE = 0,
  TAKEOFF,
  LANDING,
  FLY_TO_TARGET,
  GO_HOME,
  MOVE_FORWARD,   // 沿机头方向平移
  MOVE_BACKWARD,  // 沿机头反方向平移
  MOVE_LEFT,      // 机头左侧平移
  MOVE_RIGHT,     // 机头右侧平移
  GO_HOME_RTK,    // RTK固定解时按起飞点RTK坐标飞回并降落
};

// 飞行配置
typedef struct {
  float64_t latitude;          // 目标点纬度, 度
  float64_t longitude;         // 目标点经度, 度
  float32_t height;            // 目标点相对起飞点高度, m
  float32_t minEnrouteHeight;  // 最低航线高度(相对起飞点), m
  uint8_t speed;               // m/s
  float32_t maxDistance;       // 目标点离当前位置的最大允许距离, m
  float32_t moveDistance;      // 方向按键每次平移距离, m
  bool autoConfirmLanding;     // 降落保护触发时自动确认降落
} S_FlightConf;

// 飞行控制: 起飞/降落/飞往目标点/返航.
// 每个动作在工作线程中执行并监控完成状态; 收到新的飞行命令时,
// 当前动作的监控会被中断, 立即执行新命令(例如飞行途中按"降落"或"返航").
class FlightController {
 public:
  typedef std::function<void(const std::string &msg)> Notifier;

  FlightController();
  ~FlightController();

  int64_t Init(VehicleBase *vehicle, const S_FlightConf &conf);
  void DeInit(void);

  // 投递命令(线程安全, 非阻塞)
  void PostCmd(FlightCmd cmd);

  // 状态提示(显示到 Pilot 浮窗)
  void SetNotifier(Notifier notifier) { notifier_ = notifier; }

  static const char *CmdName(FlightCmd cmd);

 private:
  void Execute(int cmd);
  bool DoTakeoff(void);
  bool DoLanding(void);
  bool DoFlyToTarget(void);
  bool DoGoHome(void);
  // 沿机头前/后/左/右平移 moveDistance 米(保持当前高度)
  bool DoMove(FlightCmd cmd);
  // RTK 返航: 校验 RTK 固定解后按记录的起飞点坐标速度闭环飞回, 到位后降落
  bool DoGoHomeRtk(void);
  // 查询 RTK 定位解状态, 返回 false 表示非固定解(RTK 异常)
  bool QueryRtkFixed(uint8_t &solution);

  // 等待进入指定飞行模式
  bool WaitDisplayMode(uint8_t mode, uint8_t altMode, uint32_t timeoutMs);
  // 等待降落完成, 按配置在降落保护时自动确认降落
  bool WaitLanded(uint32_t timeoutS);
  // 当前位置到目标点的水平距离, m
  static double HorizontalDistance(double lat1, double lon1, double lat2,
                                   double lon2);

  E_MaicroFlightStatus GetStatus(void);
  uint8_t GetDisplayMode(void);
  void Notify(const char *fmt, ...);

 private:
  VehicleBase *vehicle_ = nullptr;
  S_FlightConf conf_;
  TaskWorker worker_;
  Notifier notifier_ = nullptr;
  // 起飞成功时记录的位置(RTK 融合坐标), RTK返航的目的地
  S_MaicroPosition homePos_;
  bool homePosValid_ = false;
};
}  // namespace maicro
