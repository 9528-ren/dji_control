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
  void HandleClient(T_DjiMopChannelHandle clientCn, uint64_t session);
  void SendLoop(T_DjiMopChannelHandle clientCn, std::atomic<bool> *alive,
                uint64_t session);
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

  // ---- 发送队列 ----
  // urgent_ 专放飞行/云台指令的 ACK(OK / ERROR:), 永远先发;
  // sendQueue_ 放 TRK/UPG/EVT 等批量回包。二者分开的理由见 QueueResponse。
  std::deque<std::string> urgent_;
  std::deque<std::string> sendQueue_;
  std::mutex sendMutex_;
  std::condition_variable sendCond_;

  // ---- 单客户端闸门 ----
  // 发送队列是**对象级共享**的, 不是每连接一份。若同时存在两个客户端会话,
  // A 的 QueueResponse 会被 B 的 SendLoop 取走, 回包发到另一条通道;
  // 两份心跳同时跑, 两路飞行指令还会抢同一个 TaskWorker 单槽。
  // App 看门狗判死后会重连, 而旧连接未必已拆完 —— 这个场景实际会发生。
  // 故约定: 只保留一个会话, 新连接到来即令旧会话退出(见 AcceptLoop)。
  // txMutex_ 串行化"检查会话有效性 + 实际发送"这一对动作。
  // 只靠 sessionSeq_ 比对是不够的: 比对通过后、SendData 真正发出前,
  // 新会话可能刚好接管, 于是旧会话又往旧通道发出一个残留包(并发测试实测到 1 个)。
  // 接管方在递增会话号后抢这把锁, 即可确保旧会话要么已发完、要么还没开始发。
  // **不能用 sendMutex_ 兼任**: 它保护队列, 而 SendData 可能阻塞数十毫秒,
  // 持它发送会把入队侧(含飞行工作线程的 QueueEvent)一起卡住。
  std::mutex txMutex_;
  std::mutex sessionMutex_;
  std::atomic<bool> clientAlive_{false};
  std::atomic<uint64_t> sessionSeq_{0};

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