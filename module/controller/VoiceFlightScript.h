#pragma once
#include <atomic>
#include <deque>
#include <mutex>
#include <string>

#include "FlightContext.h"
#include "vehicle/VehicleTypes.h"

namespace maicro {

/** 语音脚本的阶段(业务概念, 与硬件无关) */
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

/** 语音脚本的对外状态(供 App 轮询与事件推送) */
struct VoiceFlightStatus {
  VoiceFlightStage stage = VoiceFlightStage::IDLE;
  std::string request;
  const char *detail = "-";
};

/**
 * 语音飞行脚本(业务模块)。
 *
 * 三段式流程: 准备(地面起飞) -> 升高(相对爬升 1m) -> 出发(斜爬+平飞)。
 * 所有业务参数(距离/角度/速度/容差)都集中在本模块的 .cc 内, 硬件层与
 * FlightController 都不认识它们。
 *
 * **执行器共享**: 本类不持有 TaskWorker, 而是经 FlightContext 复用
 * FlightController 的单槽执行器与 generation 令牌, 从而保证按键指令
 * 能抢占正在执行的脚本(见 FlightContext 注释)。
 *
 * 线程模型:
 *  - Request()/Status() 在 MOP 收包线程调用, 只做校验与投递, 不阻塞;
 *  - Execute() 在共享执行线程调用, 内部循环下发速度矢量;
 *  - 两者以 mutex_ 保护状态, generation 令牌由 FlightController 统一管理。
 */
class VoiceFlightScript {
public:
  /// 事件输出(由 FlightController 接线为向 App 推送 TRK,VOICE_EVT 行)
  typedef std::function<void(const std::string &line)> EventSink;

  VoiceFlightScript();
  ~VoiceFlightScript();

  void Init(FlightContext *ctx) { ctx_ = ctx; }
  void SetEventSink(EventSink sink) { eventSink_ = sink; }

  /**
   * 受理一条脚本请求(MOP 线程)。
   * @param cmd    脚本命令(准备/升高/出发)
   * @param request 请求 id, 用于幂等与去重
   * @param out    受理成功时回填状态
   * @param latest true 表示"以最新为准"(可抢占), false 表示忙则拒绝
   * @param generation 由 FlightController 分配的新令牌
   * @return nullptr 表示受理; 否则为协议错误码字面量
   */
  const char *Request(int cmd, const std::string &request,
                      VoiceFlightStatus &out, bool latest,
                      uint64_t generation);

  /** 读取状态(MOP 线程); 会在发现已落地时把 PREPARED/READY 降级为 FAILED */
  VoiceFlightStatus Status();

  /** 执行脚本(共享执行线程) */
  void Execute(int cmd);

  /** 被按键等新指令抢占时调用, 把进行中的脚本标记为中断 */
  void MarkInterrupted();

  static const char *StageName(VoiceFlightStage stage);

private:
  const char *Run(int cmd, uint32_t landings);
  const char *Move(const VehicleTelemetry &origin, double north, double east,
                   double up, float32_t speed, uint32_t controlLossCount,
                   bool passThrough = false);
  void Publish(const VoiceFlightStatus &status);

  FlightContext *ctx_ = nullptr;
  EventSink eventSink_ = nullptr;

  std::mutex mutex_;
  VoiceFlightStatus status_;
  int command_ = 0;
  uint32_t roundLandings_ = 0;
  uint64_t generation_ = 0;
  std::deque<std::string> recentRequests_;
};

}  // namespace maicro
