#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <deque>
#include <mutex>
#include <string>

#include "TaskWorker.h"
#include "TrackStore.h"
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
  TRACK_RUN,      // 执行已保存的航迹(逐点串行飞)
  VOICE_PREPARE,
  VOICE_ASCEND,
  VOICE_DEPART,
};

enum class VoiceFlightStage {
  IDLE,
  PREPARING,
  PREPARED,
  ASCENDING,
  READY,
  DEPARTING,
  COMPLETE,
  FAILED,
};

struct VoiceFlightStatus {
  VoiceFlightStage stage = VoiceFlightStage::IDLE;
  std::string request;
  const char *detail = "-";
};

/** 航迹执行状态机(供 App 轮询 STAT 与 EVT 推送) */
enum class TrackState {
  IDLE = 0,  // 空闲, 未在执行
  RUNNING,   // 正在逐点执行
  DONE,      // 全部航点飞完
  ABORTED,   // 被中止(用户 ABORT / RTK 掉解 / 偏差过大 / 卡死)
  FAILED,    // 执行出错(飞控拒飞等)
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

  // ---- 航迹执行参数 ----
  // 全部带类内默认值: 即使 Pipeline 尚未从 cfg 接线, 也不会读到未初始化的垃圾值
  // (垃圾值可能让 trackMinSatellites 变成天文数字, 导致门控永远拒绝或永远放行)。
  uint16_t trackMinSatellites = 10;      // 采点/执行要求的最小可见卫星数
  uint32_t trackPointTimeoutS = 120;     // 单点飞行基础超时(秒)
  float32_t trackArriveTolerance = 2.0f; // 到位判定容差(m), 超 3 倍则中止整条航迹
  uint32_t trackGimbalSettleMs = 3000;   // 云台转到目标俯仰的等待上限(ms)
  float32_t trackAltCrossCheckM = 10.0f; // 椭球高交叉校验容差(m)
  bool trackRequireNetworkRtk = false;   // 是否额外要求负载网络RTK处于传输态
  bool trackStopOnRtkLoss = false;       // 掉解时是否用最后可信坐标强制停住
} S_FlightConf;

/** 一次航迹执行的计划(预检产物, 不含航点数据) */
typedef struct {
  std::string name;
  int startIdx = 0;      // 1-based 起始航点
  int total = 0;         // 航迹总点数
  double nearestDist = 0.0;  // 当前位置到最近航点的水平距离, m
} S_TrackRunPlan;

/** 航迹执行的实时状态(供 STAT 查询与 EVT 推送) */
typedef struct {
  TrackState state = TrackState::IDLE;
  std::string name;
  int curIdx = 0;   // 1-based, 正在飞/已完成的航点
  int total = 0;
  uint8_t rtkSol = 0;
} S_TrackRunStatus;

// 飞行控制: 起飞/降落/飞往目标点/返航/航迹执行.
// 每个动作在工作线程中执行并监控完成状态; 收到新的飞行命令时,
// 当前动作的监控会被中断, 立即执行新命令(例如飞行途中按"降落"或"返航").
class FlightController {
 public:
  typedef std::function<void(const std::string &msg)> Notifier;
  /** 航迹事件输出(由 MopCommandServer 接线为向 App 推送 TRK,EVT 行) */
  typedef std::function<void(const std::string &line)> EventSink;

  FlightController();
  ~FlightController();

  int64_t Init(VehicleBase *vehicle, const S_FlightConf &conf);
  void DeInit(void);

  // 投递命令(线程安全, 非阻塞)
  void PostCmd(FlightCmd cmd);

  // 状态提示(显示到 Pilot 浮窗)
  void SetNotifier(Notifier notifier) { notifier_ = notifier; }

  static const char *CmdName(FlightCmd cmd);

  const char *RequestVoiceFlight(FlightCmd cmd, const std::string &request,
                                 VoiceFlightStatus &out, bool latest = false);
  VoiceFlightStatus GetVoiceFlightStatus();
  static const char *VoiceStageName(VoiceFlightStage stage);

  // ==================== 航迹功能 ====================

  /** 注入航迹存储(由 Pipeline 在 Init 之后调用, 生命周期长于本对象) */
  void SetTrackStore(TrackStore *store) { track_ = store; }
  TrackStore *GetTrackStore(void) const { return track_; }

  /**
   * 注入主云台挂载位置(由 Pipeline 从配置传入, 与 GimbalController 同一来源)。
   * 航迹回放需要转动云台, 但 mountPos 是 VehicleBase 的 protected 成员,
   * 本类无法经指针访问, 故沿用 GimbalController::Init 的既有做法由外部注入。
   */
  void SetGimbalMountPos(E_MaicroMountPosition pos) { gimbalMountPos_ = pos; }

  /** 注入航迹事件输出通道 */
  void SetEventSink(EventSink sink) { eventSink_ = sink; }

  /**
   * 采集当前位姿为一个航点, 写入存储的会话缓冲。
   * 在 MOP 线程调用: 只做订阅值查询与内存写入, 不做任何飞行操作。
   *
   * @param out 成功时回填该点数据(供响应回显给 App)
   * @param errToken 失败时置为协议错误码字面量(rtk/pos/height/full/nostore)
   * @return 0 成功; 负数失败
   */
  int RecordTrackPoint(S_TrackPoint &out, const char **errToken);

  /**
   * 航迹执行的同步预检(在 MOP 线程调用, 不做任何飞行操作)。
   * 通过后才由 RequestTrackRun 投递 TRACK_RUN; 执行线程入口会再复检一次(防 TOCTOU)。
   *
   * @param reqStartIdx 1-based 指定起点; <=0 表示自动取最近点
   * @param plan 通过时回填执行计划
   * @param errToken 失败时置为协议错误码(notfound/corrupt/notinair/rtk/
   *                 rtkpoint/toofar/busy/badidx/nostore/pos)
   * @return 0 通过; 负数失败
   */
  int PreCheckTrackRun(const std::string &name, int reqStartIdx,
                       S_TrackRunPlan &plan, const char **errToken);

  /** 记录执行计划并投递 TRACK_RUN 到工作线程 */
  void RequestTrackRun(const std::string &name, int startIdx);

  /** 是否正在执行航迹(用于挡住重复 EXEC, 避免航迹被反复重启) */
  bool IsTrackRunning(void) const { return trackRunning_.load(); }

  /** 读取航迹执行状态(线程安全) */
  void GetTrackStatus(S_TrackRunStatus &out);

  /**
   * 请求中止当前航迹执行。
   *
   * **不走 PostCmd**: TaskWorker 是单槽, Post 会覆盖未取走的命令, 中止指令可能丢失。
   * 这里直接置原子标志, 执行循环每步都会检查, 保证中止永不丢失。
   */
  void AbortTrack(const char *reason);

  /** 查询 RTK 综合状态, 供 RTK 命令回显 */
  void QueryRtkStatus(uint8_t &solution, bool &fixed, uint16_t &satellites,
                      uint8_t &networkState);

 private:
  void Execute(int cmd);
  bool DoTakeoff(bool *controlObtained = nullptr, bool climbToTargetHeight = true);
  void ExecuteVoiceFlight(FlightCmd cmd);
  const char *DoVoiceFlight(FlightCmd cmd, uint32_t landings);
  const char *MoveVoiceFlight(const VoiceFlightTelemetry &origin, double north,
                              double east, double up, float32_t speed,
                              uint32_t controlLossCount, bool passThrough = false);
  void PublishVoiceStatus(const VoiceFlightStatus &status);
  bool DoLanding(void);
  bool DoFlyToTarget(void);
  // 返航: 按起飞时记录的起飞点坐标, 用地面坐标系(NEU)速度闭环飞回, **确认水平到位后**
  // 才降落。不区分 RTK/GPS —— 两者都执行, 仅按当前定位解质量放宽到位判据。
  bool DoGoHome(void);
  // 原生返航回退路径: 仅当起飞点未记录(如程序启动时飞机已在空中)时使用,
  // 交由飞控自主返航降落, 本函数只监控到落地为止, 不自行宣告"已到达"。
  bool DoGoHomeNative(void);
  // 沿机头前/后/左/右平移 moveDistance 米(保持当前高度)
  bool DoMove(FlightCmd cmd);
  // RTK 返航按钮: 与 DoGoHome 行为完全一致(不区分定位源), 仅为保留既有命令与按钮
  bool DoGoHomeRtk(void);
  // 航迹执行: 从最近点(或指定点)起逐点串行飞完剩余航点
  bool DoRunTrack(void);
  // 校验一个位置是否可作为返航起飞点(排除无定位时的垃圾坐标)。
  // 判据: 经纬度非近零、在合法范围内、可见卫星数达标。
  bool IsHomePositionSane(const S_MaicroPosition &pos);
  // 查询 RTK 定位解状态, 返回 false 表示非固定解(RTK 异常)
  bool QueryRtkFixed(uint8_t &solution);
  // 连续 times 次、间隔 intervalMs 全为固定解且卫星数达标才返回 true
  bool WaitRtkStable(int times, uint32_t intervalMs, uint8_t &lastSol);
  // 推送一行航迹事件给 App
  void PublishTrackEvent(const char *stateName);

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
  EventSink eventSink_ = nullptr;
  // 起飞成功时记录的位置(RTK融合坐标), RTK返航的目的地
  S_MaicroPosition homePos_;
  bool homePosValid_ = false;

  std::mutex voiceMutex_;
  VoiceFlightStatus voiceStatus_;
  FlightCmd voiceCommand_ = FlightCmd::NONE;
  uint32_t voiceRoundLandings_ = 0;
  uint64_t voiceGeneration_ = 0;
  std::deque<std::string> voiceRecentRequests_;

  // ---- 航迹执行状态 ----
  TrackStore *track_ = nullptr;
  // 主云台挂载位置(航迹回放转云台用), 由 Pipeline 注入
  E_MaicroMountPosition gimbalMountPos_ = MAICRO_MOUNT_POSITION_PAYLOAD_PORT_NO1;
  std::atomic<bool> abortTrack_{false};    // 中止请求(MOP 线程置位, 执行线程轮询)
  std::atomic<bool> trackRunning_{false};  // 正在执行航迹
  std::atomic<int> trackState_{(int)TrackState::IDLE};
  std::atomic<int> trackIdx_{0};
  std::atomic<int> trackTotal_{0};

  mutable std::mutex trackMutex_;  // 保护下面的字符串与计划字段
  std::string trackName_;          // 当前/最近一次执行的航迹名
  std::string pendingName_;        // 待执行的航迹名(RequestTrackRun 设置)
  int pendingStartIdx_ = 0;
};
}  // namespace maicro
