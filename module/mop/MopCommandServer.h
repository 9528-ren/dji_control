#pragma once
#include "controller/FlightController.h"
#include "controller/GimbalController.h"
#include "dji_mop_channel.h"
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace maicro {
class MopCommandServer {
public:
  enum : uint16_t { CHANNEL_ID = 8888 };
  MopCommandServer();
  ~MopCommandServer();
  int Init(GimbalController *gimbal, FlightController *flight);
  void DeInit(void);

private:
  void InitAndAccept(void);
  void AcceptLoop(void);
  void HandleClient(T_DjiMopChannelHandle clientCn);
  void SendLoop(T_DjiMopChannelHandle clientCn, std::atomic<bool> *alive);
  void DispatchCommand(const std::string &cmd);
  void QueueResponse(const std::string &resp);

  GimbalController *gimbal_ = nullptr;
  FlightController *flight_ = nullptr;
  T_DjiMopChannelHandle listenCn_ = nullptr;
  std::atomic<bool> running_{false};
  std::shared_ptr<std::thread> acceptThread_ = nullptr;

  std::deque<std::string> sendQueue_;
  std::mutex sendMutex_;
};
} // namespace maicro