#pragma once
#include "controller/FlightController.h"
#include "controller/GimbalController.h"
#include "controller/TrackStore.h"
#include "dji_mop_channel.h"
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace maicro {
class MopCommandServer {
public:
  enum : uint16_t { CHANNEL_ID = 8888 };
  MopCommandServer();
  ~MopCommandServer();

  /**
   * @param track 航迹存储, 可为 nullptr(此时所有 TRK 命令回 ERROR:nostore)。
   *              生命周期必须长于本对象。
   */
  int Init(GimbalController *gimbal, FlightController *flight,
           TrackStore *track);
  void DeInit(void);

  /**
   * 把一行主动推送事件放入发送队列(供 FlightController 的航迹事件回调使用)。
   * 与 QueueResponse 同一队列, 但**不校验 running_**, 因为航迹事件可能在
   * DeInit 竞态窗口内产生; 队列在连接关闭时会被清空, 不会泄漏到下一个客户端。
   */
  void QueueEvent(const std::string &line);

private:
  void InitAndAccept(void);
  void AcceptLoop(void);
  void HandleClient(T_DjiMopChannelHandle clientCn);
  void SendLoop(T_DjiMopChannelHandle clientCn, std::atomic<bool> *alive);
  void DispatchCommand(const std::string &cmd);
  void QueueResponse(const std::string &resp);

  /** 处理 TRK,<rid>,<VERB>[,<arg>...] 形式的航迹命令 */
  void HandleTrackCommand(const std::string &cmd);

  GimbalController *gimbal_ = nullptr;
  FlightController *flight_ = nullptr;
  TrackStore *track_ = nullptr;
  T_DjiMopChannelHandle listenCn_ = nullptr;
  std::atomic<bool> running_{false};
  std::shared_ptr<std::thread> acceptThread_ = nullptr;

  std::deque<std::string> sendQueue_;
  std::mutex sendMutex_;
};
} // namespace maicro