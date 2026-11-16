#include "MopCommandServer.h"
#include "MaicroLogger.inl"
#include <chrono>
#include <mutex>
#include <thread>

namespace maicro {
namespace {
const uint64_t kHeartbeatIntervalMs = 15000;
const char kHeartbeatText[] = "HB\n";

const uint32_t kMaxConsecutiveRecvErr = 100;

const uint32_t kSendDrainBatch = 16;
} // namespace

MopCommandServer::MopCommandServer() {}
MopCommandServer::~MopCommandServer() { DeInit(); }

int MopCommandServer::Init(GimbalController *gimbal, FlightController *flight) {
  gimbal_ = gimbal;
  flight_ = flight;
  running_ = true;

  acceptThread_ =
      std::make_shared<std::thread>(&MopCommandServer::InitAndAccept, this);
  return 0;
}

void MopCommandServer::InitAndAccept(void) {
  int initTries = 0;
  while (running_ && initTries < 5) {
    T_DjiReturnCode ret = DjiMopChannel_Init();
    if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      MAICRO_INFO("mop channel init success");
      break;
    }
    initTries++;
    MAICRO_WARN("mop channel init retry(%d) err=0x%08X", initTries, ret);
    sleep(2);
  }
  if (initTries >= 5) {
    MAICRO_ERROR("mop channel init failed after %d tries continue anyway",
                 initTries);
  }

  while (running_) {
    T_DjiReturnCode ret =
        DjiMopChannel_Create(&listenCn_, DJI_MOP_CHANNEL_TRANS_RELIABLE);
    if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      ret = DjiMopChannel_Bind(listenCn_, CHANNEL_ID);
      if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        MAICRO_INFO("mop cmd server started channel=%u", CHANNEL_ID);
        AcceptLoop();
        return;
      } else {
        MAICRO_ERROR("mop cmd server: bind error 0x%08X retry...", ret);
        DjiMopChannel_Destroy(listenCn_);
        listenCn_ = nullptr;
      }
    } else {
      MAICRO_WARN("mop cmd server: channel create retry err=0x%08X", ret);
    }
    sleep(2);
  }
}

void MopCommandServer::DeInit(void) {
  running_ = false;
  if (listenCn_) {
    DjiMopChannel_Close(listenCn_);
    DjiMopChannel_Destroy(listenCn_);
    listenCn_ = nullptr;
  }
  if (acceptThread_ && acceptThread_->joinable()) {
    acceptThread_->join();
  }
}

void MopCommandServer::AcceptLoop(void) {
  while (running_) {
    T_DjiMopChannelHandle clientCn = nullptr;
    T_DjiReturnCode ret = DjiMopChannel_Accept(listenCn_, &clientCn);
    if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && clientCn) {
      MAICRO_INFO("mop cmd server: client connected");
      std::thread(&MopCommandServer::HandleClient, this, clientCn).detach();
    } else {
      usleep(500 * 1000);
    }
  }
}

void MopCommandServer::HandleClient(T_DjiMopChannelHandle clientCn) {
  char recvBuf[1024];
  uint32_t realLen = 0;
  std::string lineBuf;
  uint32_t timeoutCount = 0;
  uint32_t errBurst = 0;
  std::atomic<bool> alive{true};

  MAICRO_INFO("mop cmd server: handler start sendLoop+hb every %llums",
              (unsigned long long)kHeartbeatIntervalMs);

  std::thread sendThread(&MopCommandServer::SendLoop, this, clientCn, &alive);

  while (running_) {
    T_DjiReturnCode ret = DjiMopChannel_RecvData(clientCn, (uint8_t *)recvBuf,
                                                 sizeof(recvBuf), &realLen);

    if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      timeoutCount = 0;
      errBurst = 0;
      if (realLen > 0) {
        lineBuf.append(recvBuf, realLen);
        size_t pos;
        while ((pos = lineBuf.find('\n')) != std::string::npos) {
          std::string cmd = lineBuf.substr(0, pos);
          lineBuf.erase(0, pos + 1);
          if (!cmd.empty() && cmd.back() == '\r')
            cmd.pop_back();
          if (!cmd.empty()) {
            auto t0 = std::chrono::steady_clock::now();
            uint64_t ms0 =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    t0.time_since_epoch())
                    .count();
            MAICRO_INFO("mop [recv] cmd=%s t=%llums", cmd.c_str(),
                        (unsigned long long)ms0);
            DispatchCommand(cmd);
          }
        }
      }
    } else if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT) {
      if (++timeoutCount % 100 == 1) {
        MAICRO_INFO("mop cmd server: recv timeout x%u link alive keep waiting",
                    timeoutCount);
      }
    } else if (ret == DJI_ERROR_MOP_CHANNEL_MODULE_CODE_CONNECTION_CLOSE) {
      MAICRO_INFO("mop cmd server: peer closed connection ret=0x%08X", ret);
      break;
    } else {
      if (++errBurst == 1 || errBurst % 50 == 0) {
        MAICRO_WARN(
            "mop cmd server: recv err burst ret=0x%08X count=%u (tolerating)",
            ret, errBurst);
      }
      if (errBurst >= kMaxConsecutiveRecvErr) {
        MAICRO_ERROR("mop cmd server: too many consecutive recv errors (%u) "
                     "give up ret=0x%08X",
                     errBurst, ret);
        break;
      }
    }
  }

  alive = false;
  if (sendThread.joinable())
    sendThread.join();

  {
    std::lock_guard<std::mutex> lk(sendMutex_);
    sendQueue_.clear();
  }

  DjiMopChannel_Close(clientCn);
  DjiMopChannel_Destroy(clientCn);
  MAICRO_INFO("mop cmd server: handler exit");
}

void MopCommandServer::SendLoop(T_DjiMopChannelHandle clientCn,
                                std::atomic<bool> *alive) {
  int hbCount = 0;
  auto lastHb = std::chrono::steady_clock::now();

  auto sendOne = [&](const std::string &data) -> bool {
    uint32_t sentLen = 0;
    auto s0 = std::chrono::steady_clock::now();
    T_DjiReturnCode sret = DjiMopChannel_SendData(
        clientCn, (uint8_t *)data.c_str(), data.size(), &sentLen);
    auto s1 = std::chrono::steady_clock::now();
    auto sendCostMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(s1 - s0).count();
    if (sendCostMs > 10 || data != "HB\n")
      MAICRO_INFO("mop SendData cost=%lldms ret=0x%08X sent=%u",
                  (long long)sendCostMs, sret, sentLen);
    if (sret == DJI_ERROR_MOP_CHANNEL_MODULE_CODE_CONNECTION_CLOSE) {
      MAICRO_INFO("mop sendLoop: connection close detected");
      alive->store(false);
      return false;
    }
    if (sret == DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT) {
      MAICRO_WARN("mop sendLoop: send timeout ret=0x%08X", sret);
      usleep(50 * 1000);
      return true;
    }
    if (sret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      MAICRO_WARN("mop sendLoop: send err ret=0x%08X", sret);
      usleep(50 * 1000);
    }
    return true;
  };

  while (alive->load() && running_) {
    bool drained = false;
    {
      std::lock_guard<std::mutex> lk(sendMutex_);
      drained = sendQueue_.empty();
    }

    while (!drained && alive->load() && running_) {
      std::string item;
      {
        std::lock_guard<std::mutex> lk(sendMutex_);
        if (sendQueue_.empty()) {
          drained = true;
          break;
        }
        item = std::move(sendQueue_.front());
        sendQueue_.pop_front();
      }
      {
        auto t0 = std::chrono::steady_clock::now();
        uint64_t ms0 = std::chrono::duration_cast<std::chrono::milliseconds>(
                           t0.time_since_epoch())
                           .count();
        MAICRO_INFO("mop [send] data=%s t=%llums", item.c_str(),
                    (unsigned long long)ms0);
      }
      if (!sendOne(item))
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsedMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - lastHb)
            .count();
    if (elapsedMs >= kHeartbeatIntervalMs) {
      bool queueHasCmds;
      {
        std::lock_guard<std::mutex> lk(sendMutex_);
        queueHasCmds = !sendQueue_.empty();
      }
      lastHb = now;
      if (!queueHasCmds) {
        hbCount++;
        std::string hb(kHeartbeatText);
        if (!sendOne(hb))
          return;
        if (hbCount % 10 == 0) {
          uint32_t pending;
          {
            std::lock_guard<std::mutex> lk(sendMutex_);
            pending = sendQueue_.size();
          }
          MAICRO_INFO("mop sendLoop heartbeat #%d pending=%u", hbCount,
                      pending);
        }
      }
    }

    {
      bool hasPending;
      std::lock_guard<std::mutex> lk(sendMutex_);
      hasPending = !sendQueue_.empty();
      usleep(hasPending ? 1 * 1000 : 30 * 1000);
    }
  }
}

void MopCommandServer::QueueResponse(const std::string &resp) {
  std::lock_guard<std::mutex> lk(sendMutex_);
  sendQueue_.push_back(resp);
}

void MopCommandServer::DispatchCommand(const std::string &cmd) {
  MAICRO_INFO("mop cmd: %s", cmd.c_str());

  if (cmd == "HEARTBEAT") {
    QueueResponse("OK\n");
    return;
  }
  if (cmd == "TAKEOFF") {
    flight_->PostCmd(FlightCmd::TAKEOFF);
  } else if (cmd == "LANDING") {
    flight_->PostCmd(FlightCmd::LANDING);
  } else if (cmd == "PITCH_UP") {
    gimbal_->PostCmd(GimbalCmd::PITCH_UP);
  } else if (cmd == "PITCH_DOWN") {
    gimbal_->PostCmd(GimbalCmd::PITCH_DOWN);
  } else if (cmd == "RECENTER") {
    gimbal_->PostCmd(GimbalCmd::RECENTER);
  } else if (cmd == "FLY_TO_TARGET") {
    flight_->PostCmd(FlightCmd::FLY_TO_TARGET);
  } else if (cmd == "MOVE_FORWARD") {
    flight_->PostCmd(FlightCmd::MOVE_FORWARD);
  } else if (cmd == "MOVE_BACKWARD") {
    flight_->PostCmd(FlightCmd::MOVE_BACKWARD);
  } else if (cmd == "MOVE_LEFT") {
    flight_->PostCmd(FlightCmd::MOVE_LEFT);
  } else if (cmd == "MOVE_RIGHT") {
    flight_->PostCmd(FlightCmd::MOVE_RIGHT);
  } else if (cmd == "GO_HOME") {
    flight_->PostCmd(FlightCmd::GO_HOME);
  } else if (cmd == "GO_HOME_RTK") {
    flight_->PostCmd(FlightCmd::GO_HOME_RTK);
  } else {
    QueueResponse("ERROR:unknown\n");
    return;
  }
  QueueResponse("OK\n");
}
} // namespace maicro