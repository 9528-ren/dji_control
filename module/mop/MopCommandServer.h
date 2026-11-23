#pragma once
#include "controller/FlightController.h"
#include "controller/GimbalController.h"
#include "controller/TrackStore.h"
#include "dji_mop_channel.h"
#include <atomic>
#include <condition_variable>
#include <cstdint>
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

  /**
   * 处理 UPG,<VERB>[,args] 形式的分片升级命令。
   *
   * 与飞行/航迹完全隔离: 响应行统一 "UPG," 前缀, 不影响指令 ACK 语义。
   * 数据落到 kUpgradeDir 下的 staging 文件, UPG,END 校验通过后由
   * upgrade::CommitPending 落成 pending, 下次启动自动替换并 exec。
   */
  void HandleUpgradeCommand(const std::string &cmd);
  void CloseUpgradeFile();

  GimbalController *gimbal_ = nullptr;
  FlightController *flight_ = nullptr;
  TrackStore *track_ = nullptr;
  T_DjiMopChannelHandle listenCn_ = nullptr;
  std::atomic<bool> running_{false};
  std::shared_ptr<std::thread> acceptThread_ = nullptr;

  std::deque<std::string> sendQueue_;
  std::mutex sendMutex_;
  std::condition_variable sendCond_;

  // ---- 分片升级状态(只在 MOP 收包线程读写, 无需加锁) ----
  int upgFd_ = -1;
  uint64_t upgSize_ = 0;      // 目标文件总字节
  uint64_t upgChunk_ = 0;     // 每片原始字节数
  uint64_t upgSeqTotal_ = 0;  // 总分片数
  uint64_t upgSeenCount_ = 0; // 已收到(去重后)的分片数
  std::string upgMd5_;        // 期望 MD5
  std::vector<uint8_t> upgSeen_; // 分片到达位图
};
} // namespace maicro