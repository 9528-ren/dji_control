#include "MopCommandServer.h"
#include "MaicroLogger.inl"
#include "controller/CommandTable.h"
#include "mop/Md5.h"
#include "mop/SelfUpgrade.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

namespace maicro {
namespace {
const uint64_t kHeartbeatIntervalMs = 15000;
const char kHeartbeatText[] = "HB\n";

const uint32_t kMaxConsecutiveRecvErr = 100;

const size_t kMaxSendQueue = 1024;
const size_t kMaxCommandBytes = 4096;

const uint32_t kSendDrainBatch = 16;
} // namespace

MopCommandServer::MopCommandServer() {}
MopCommandServer::~MopCommandServer() { DeInit(); }

int MopCommandServer::Init(GimbalController *gimbal, FlightController *flight,
                           TrackStore *track) {
  gimbal_ = gimbal;
  flight_ = flight;
  track_ = track;
  running_ = true;

  // 航迹事件(TRK,EVT,...)经发送队列推给 App。
  // 注意: 此回调运行在飞行工作线程, QueueEvent 只加锁入队, 不做阻塞操作。
  if (flight_) {
    flight_->SetEventSink(
        [this](const std::string &line) { QueueEvent(line); });
  }

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
        if (lineBuf.size() + realLen > kMaxCommandBytes) {
          MAICRO_WARN(
              "mop command exceeds maximum length; dropping buffered input");
          lineBuf.clear();
          continue;
        }
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
            // 升级分片每秒几十条, 逐条打日志会拖慢传输, 这里静默
            if (cmd.compare(0, 9, "UPG,DATA,") != 0) {
              MAICRO_INFO("mop [recv] cmd=%s t=%llums", cmd.c_str(),
                          (unsigned long long)ms0);
            }
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
    std::string item;
    {
      std::unique_lock<std::mutex> lk(sendMutex_);
      auto now = std::chrono::steady_clock::now();
      auto nextHb = lastHb + std::chrono::milliseconds(kHeartbeatIntervalMs);
      sendCond_.wait_until(lk, nextHb, [&] {
        return !sendQueue_.empty() || !alive->load() || !running_;
      });
      if (!alive->load() || !running_)
        return;
      if (sendQueue_.empty()) {
        lastHb = std::chrono::steady_clock::now();
        lk.unlock();
        if (!sendOne(std::string(kHeartbeatText)))
          return;
        continue;
      }
      item = std::move(sendQueue_.front());
      sendQueue_.pop_front();
    }
    {
      auto t0 = std::chrono::steady_clock::now();
      uint64_t ms0 = std::chrono::duration_cast<std::chrono::milliseconds>(
                         t0.time_since_epoch())
                         .count();
      // 分片 ACK 同样静默, 避免 4000+ 条日志拖慢升级
      if (item.compare(0, 6, "UPG,D,") != 0) {
        MAICRO_INFO("mop [send] data=%s t=%llums", item.c_str(),
                    (unsigned long long)ms0);
      }
    }
    if (!sendOne(item))
      return;
  }
}

void MopCommandServer::QueueResponse(const std::string &resp) {
  std::lock_guard<std::mutex> lk(sendMutex_);
  if (sendQueue_.size() >= kMaxSendQueue) {
    MAICRO_WARN("mop send queue full; response dropped");
    return;
  }
  sendQueue_.push_back(resp);
  sendCond_.notify_one();
}

void MopCommandServer::QueueEvent(const std::string &line) {
  // 与 QueueResponse 同一队列。不校验 running_: 航迹事件可能在 DeInit
  // 竞态窗口内 产生, 此时入队无害(连接关闭时队列会被清空,
  // 不会串到下一个客户端)。
  std::lock_guard<std::mutex> lk(sendMutex_);
  if (sendQueue_.size() >= kMaxSendQueue) {
    MAICRO_WARN("mop send queue full; event dropped");
    return;
  }
  sendQueue_.push_back(line);
  sendCond_.notify_one();
}

void MopCommandServer::DispatchCommand(const std::string &cmd) {
  if (cmd.compare(0, 9, "UPG,DATA,") != 0) {
    MAICRO_INFO("mop cmd: %s", cmd.c_str());
  }

  // 航迹命令走独立协议(TRK,<rid>,<VERB>...), 与 12 条飞行指令完全隔离:
  // 飞行指令的 ACK 语义(只认 OK/ERROR: 前缀、发送前清队列、不重发)一字不动。
  if (cmd.compare(0, 4, "TRK,") == 0) {
    HandleTrackCommand(cmd);
    return;
  }

  // 分片升级(UPG,...) 同样走独立前缀, 不影响飞行指令 ACK
  if (cmd.compare(0, 4, "UPG,") == 0) {
    HandleUpgradeCommand(cmd);
    return;
  }

  if (cmd == "HEARTBEAT") {
    QueueResponse("OK\n");
    return;
  }

  // ??????(? CommandTable.h)?????? 12 ? else-if ?????,
  // ? WidgetHandler ??? switch ?????, ???????????
  const CommandSpec *spec = FindCommandByName(cmd);
  if (!spec) {
    QueueResponse("ERROR:unknown\n");
    return;
  }
  if (spec->target == CmdTarget::FLIGHT) {
    flight_->PostCmd(static_cast<FlightCmd>(spec->code));
  } else {
    gimbal_->PostCmd(static_cast<GimbalCmd>(spec->code));
  }
  QueueResponse("OK\n");
}

namespace {
// 按逗号切分, 保留空字段(STAT 的 name 可能为空, App 侧依赖固定字段位)
std::vector<std::string> SplitComma(const std::string &s) {
  std::vector<std::string> out;
  size_t start = 0;
  while (true) {
    size_t p = s.find(',', start);
    if (p == std::string::npos) {
      out.push_back(s.substr(start));
      break;
    }
    out.push_back(s.substr(start, p - start));
    start = p + 1;
  }
  return out;
}

// 航迹状态的线上短标记, 与 EVT 事件用同一套词汇(RUN/DONE/ABORT/ERR)
const char *TrackStateName(TrackState s) {
  switch (s) {
  case TrackState::RUNNING:
    return "RUN";
  case TrackState::DONE:
    return "DONE";
  case TrackState::ABORTED:
    return "ABORT";
  case TrackState::FAILED:
    return "ERR";
  case TrackState::IDLE:
  default:
    return "IDLE";
  }
}
} // namespace

// 处理 TRK,<rid>,<VERB>[,<arg>...] 航迹命令。
// 与 12 条飞行指令完全隔离: 响应行一律带 TRK,<rid>, 前缀, 飞行指令的 ACK
// 语义不受影响。 所有文件 IO 都在 MOP 线程(本函数所在线程)完成,
// 不进飞行工作线程。
void MopCommandServer::HandleTrackCommand(const std::string &cmd) {
  std::vector<std::string> f = SplitComma(cmd);
  std::string rid = (f.size() > 1) ? f[1] : "0";

  auto emit = [&](const std::string &payload) {
    QueueResponse("TRK," + rid + "," + payload + "\n");
  };
  auto emitErr = [&](const char *code) {
    QueueResponse("TRK," + rid + ",ERROR:" + code + "\n");
  };

  if (f.size() < 3) {
    emitErr("badcmd");
    return;
  }
  const std::string &verb = f[2];
  char buf[256];

  // HELLO 只用于探测服务端是否已升级, 不依赖 track_/flight_, 最先处理
  if (verb == "HELLO") {
    emit("OK,1");
    return;
  }
  if (verb == "WIDGET_LATEST") {
    if (!flight_) {
      emitErr("flightctrl");
      return;
    }
    if (f.size() != 4) {
      emitErr("badcmd");
      return;
    }
    FlightCmd action;
    if (f[3] == "TAKEOFF")
      action = FlightCmd::TAKEOFF;
    else if (f[3] == "LANDING")
      action = FlightCmd::LANDING;
    else if (f[3] == "FLY_TO_TARGET")
      action = FlightCmd::FLY_TO_TARGET;
    else if (f[3] == "MOVE_FORWARD")
      action = FlightCmd::MOVE_FORWARD;
    else if (f[3] == "MOVE_BACKWARD")
      action = FlightCmd::MOVE_BACKWARD;
    else if (f[3] == "MOVE_LEFT")
      action = FlightCmd::MOVE_LEFT;
    else if (f[3] == "MOVE_RIGHT")
      action = FlightCmd::MOVE_RIGHT;
    else if (f[3] == "GO_HOME")
      action = FlightCmd::GO_HOME;
    else if (f[3] == "GO_HOME_RTK")
      action = FlightCmd::GO_HOME_RTK;
    else {
      emitErr("badcmd");
      return;
    }
    flight_->PostCmd(action);
    emit("OK,QUEUED");
    return;
  }
  if (verb == "VOICE" || verb == "VOICE_LATEST") {
    const bool latest = (verb == "VOICE_LATEST");
    if (!flight_) {
      emitErr("flightctrl");
      return;
    }
    VoiceFlightStatus status;
    if (f.size() == 4 && f[3] == "STATUS") {
      status = flight_->GetVoiceFlightStatus();
    } else {
      if (f.size() != 5 || f[4].size() != 32 ||
          f[4].find_first_not_of("0123456789abcdef") != std::string::npos) {
        emitErr("badrequest");
        return;
      }
      FlightCmd action;
      if (f[3] == "PREPARE")
        action = FlightCmd::VOICE_PREPARE;
      else if (f[3] == "ASCEND")
        action = FlightCmd::VOICE_ASCEND;
      else if (f[3] == "DEPART")
        action = FlightCmd::VOICE_DEPART;
      else {
        emitErr("badcmd");
        return;
      }
      if (const char *error =
              flight_->RequestVoiceFlight(action, f[4], status, latest)) {
        emitErr(error);
        return;
      }
    }
    emit(std::string(latest ? "OK,VOICE2," : "OK,VOICE1,") +
         FlightController::VoiceStageName(status.stage) + "," +
         (status.request.empty() ? "-" : status.request) + "," + status.detail);
    return;
  }
  if (!track_ || !flight_) {
    emitErr("nostore");
    return;
  }

  if (verb == "RTK") {
    uint8_t sol = 0, net = 0;
    bool fixed = false;
    uint16_t sat = 0;
    flight_->QueryRtkStatus(sol, fixed, sat, net);
    snprintf(buf, sizeof(buf), "OK,%u,%d,%u,%u", (unsigned)sol, fixed ? 1 : 0,
             (unsigned)sat, (unsigned)net);
    emit(buf);
    return;
  }

  if (verb == "BEGIN" || verb == "CLEAR") {
    track_->BeginSession();
    snprintf(buf, sizeof(buf), "OK,%d", track_->SessionPointCount());
    emit(buf);
    return;
  }

  if (verb == "CAPTURE") {
    S_TrackPoint pt{}; // 值初始化, 全零
    const char *errTok = "pos";
    if (flight_->RecordTrackPoint(pt, &errTok) != 0) {
      emitErr(errTok);
      return;
    }
    snprintf(buf, sizeof(buf), "OK,%u,%.7f,%.7f,%.2f,%.2f,%u", (unsigned)pt.idx,
             pt.lat, pt.lon, (double)pt.relH, (double)pt.gPitch,
             (unsigned)pt.rtkSol);
    emit(buf);
    return;
  }

  if (verb == "SAVE") {
    if (f.size() < 4) {
      emitErr("badname");
      return;
    }
    std::string name = TrackStore::DecodeName(f[3]);
    bool force = (f.size() >= 5 && f[4] == "FORCE");
    int r = track_->SaveTrack(name, force);
    if (r == 0) {
      snprintf(buf, sizeof(buf), "OK,%d", track_->SessionPointCount());
      emit(buf);
    } else if (r == -1) {
      emitErr("badname");
    } else if (r == -2) {
      emitErr("empty");
    } else if (r == -3) {
      emitErr("exists");
    } else {
      emitErr("io");
    }
    return;
  }

  if (verb == "LIST") {
    int off = (f.size() > 3) ? atoi(f[3].c_str()) : 0;
    int cnt = (f.size() > 4) ? atoi(f[4].c_str()) : 20;
    if (off < 0)
      off = 0;
    if (cnt <= 0 || cnt > 20)
      cnt = 20; // 每页上限 20, 保证单行不超长
    std::vector<S_TrackItem> items;
    if (track_->ListTracks(items) != 0) {
      emitErr("io");
      return;
    }
    int total = (int)items.size();
    int ret = 0;
    for (int i = off; i < total && i < off + cnt; ++i)
      ret++;
    snprintf(buf, sizeof(buf), "LISTOK,%d,%d,%d", total, off, ret);
    emit(buf);
    for (int i = off; i < total && i < off + cnt; ++i) {
      snprintf(buf, sizeof(buf), "ITEM,%s,%d,%lld",
               TrackStore::EncodeName(items[i].name).c_str(),
               items[i].pointCount, (long long)items[i].createdAt);
      emit(buf);
    }
    emit("LISTEND");
    return;
  }

  if (verb == "GET") {
    if (f.size() < 4) {
      emitErr("badname");
      return;
    }
    std::string name = TrackStore::DecodeName(f[3]);
    int off = (f.size() > 4) ? atoi(f[4].c_str()) : 0;
    int cnt = (f.size() > 5) ? atoi(f[5].c_str()) : 20;
    if (off < 0)
      off = 0;
    if (cnt <= 0 || cnt > 20)
      cnt = 20;
    S_Track t;
    int lr = track_->LoadTrack(name, t);
    if (lr == -2) {
      emitErr("notfound");
      return;
    }
    if (lr != 0) {
      emitErr("corrupt"); // 校验和/字段损坏, 绝不返回部分数据
      return;
    }
    int total = (int)t.pts.size();
    int ret = 0;
    for (int i = off; i < total && i < off + cnt; ++i)
      ret++;
    snprintf(buf, sizeof(buf), "GETOK,%s,%d,%d,%d",
             TrackStore::EncodeName(t.name).c_str(), total, off, ret);
    emit(buf);
    for (int i = off; i < total && i < off + cnt; ++i) {
      const S_TrackPoint &p = t.pts[i];
      snprintf(buf, sizeof(buf), "PT,%u,%.7f,%.7f,%.2f,%.3f,%.2f,%.2f,%.2f,%u",
               (unsigned)p.idx, p.lat, p.lon, (double)p.relH, p.altEll,
               (double)p.gPitch, (double)p.gYaw, (double)p.dYaw,
               (unsigned)p.rtkSol);
      emit(buf);
    }
    emit("GETEND");
    return;
  }

  if (verb == "DEL") {
    if (f.size() < 4) {
      emitErr("badname");
      return;
    }
    std::string name = TrackStore::DecodeName(f[3]);
    if (flight_->IsTrackRunning()) {
      emitErr("busy"); // 执行中不允许删除正在跑的航迹
      return;
    }
    int r = track_->DeleteTrack(name);
    if (r == 0) {
      emit("OK,1");
    } else if (r == -2) {
      emitErr("notfound");
    } else if (r == -1) {
      emitErr("badname");
    } else {
      emitErr("io");
    }
    return;
  }

  if (verb == "EXEC") {
    if (f.size() < 4) {
      emitErr("badname");
      return;
    }
    std::string name = TrackStore::DecodeName(f[3]);
    int startIdx = (f.size() > 4) ? atoi(f[4].c_str()) : 0; // <=0 自动取最近点
    S_TrackRunPlan plan;
    const char *errTok = "notfound";
    if (flight_->PreCheckTrackRun(name, startIdx, plan, &errTok) != 0) {
      emitErr(errTok);
      return;
    }
    flight_->RequestTrackRun(plan.name, plan.startIdx);
    snprintf(buf, sizeof(buf), "OK,%d,%d,%.1f", plan.startIdx, plan.total,
             plan.nearestDist);
    emit(buf);
    return;
  }

  if (verb == "ABORT") {
    if (!flight_->IsTrackRunning()) {
      emit("OK,0,idle");
      return;
    }
    flight_->AbortTrack("user");
    S_TrackRunStatus st;
    flight_->GetTrackStatus(st);
    snprintf(buf, sizeof(buf), "OK,%d,user", st.curIdx);
    emit(buf);
    return;
  }

  if (verb == "STAT") {
    S_TrackRunStatus st;
    flight_->GetTrackStatus(st);
    snprintf(buf, sizeof(buf), "OK,%s,%s,%d,%d,%u", TrackStateName(st.state),
             TrackStore::EncodeName(st.name).c_str(), st.curIdx, st.total,
             (unsigned)st.rtkSol);
    emit(buf);
    return;
  }

  emitErr("unknown");
}

// ==================== 分片升级 ====================
namespace {

// Base64 解码(忽略非法字符与填充); 分片内容只来自 App 侧标准 Base64
bool B64Decode(const std::string &in, std::vector<uint8_t> *out) {
  out->clear();
  out->reserve(in.size() * 3 / 4 + 3);
  int val = 0;
  int bits = -8;
  for (char ch : in) {
    int d;
    if (ch >= 'A' && ch <= 'Z')
      d = ch - 'A';
    else if (ch >= 'a' && ch <= 'z')
      d = ch - 'a' + 26;
    else if (ch >= '0' && ch <= '9')
      d = ch - '0' + 52;
    else if (ch == '+')
      d = 62;
    else if (ch == '/')
      d = 63;
    else
      continue; // '=' 或空白
    val = (val << 6) + d;
    bits += 6;
    if (bits >= 0) {
      out->push_back((uint8_t)((val >> bits) & 0xFF));
      bits -= 8;
    }
  }
  return !out->empty();
}

std::string ToLowerHex(std::string s) {
  for (auto &c : s) {
    if (c >= 'A' && c <= 'F')
      c = (char)(c - 'A' + 'a');
  }
  return s;
}

} // namespace

void MopCommandServer::CloseUpgradeFile() {
  if (upgFd_ >= 0) {
    fsync(upgFd_);
    close(upgFd_);
    upgFd_ = -1;
  }
}

void MopCommandServer::HandleUpgradeCommand(const std::string &cmd) {
  std::vector<std::string> f = SplitComma(cmd);
  std::string verb = f.size() > 1 ? f[1] : "";

  // 查询当前运行程序的信息: UPG,INFO -> UPG,I,OK,<md5>,<size>
  if (verb == "INFO") {
    std::string self = upgrade::SelfExePath();
    std::string md5;
    uint64_t size = 0;
    if (self.empty() || !upgrade::Md5File(self, &md5)) {
      QueueResponse("UPG,ERR,self\n");
      return;
    }
    struct stat st;
    if (stat(self.c_str(), &st) == 0) {
      size = (uint64_t)st.st_size;
    }
    QueueResponse("UPG,I,OK," + md5 + "," + std::to_string(size) + "\n");
    return;
  }

  // 开始: UPG,BEGIN,<size>,<md5>,<chunkBytes>
  if (verb == "BEGIN") {
    if (f.size() < 5) {
      QueueResponse("UPG,ERR,args\n");
      return;
    }
    uint64_t size = strtoull(f[2].c_str(), nullptr, 10);
    std::string md5 = ToLowerHex(f[3]);
    uint64_t chunk = strtoull(f[4].c_str(), nullptr, 10);
    if (size == 0 || size > 256ull * 1024 * 1024 || md5.size() != 32 ||
        chunk < 256 || chunk > 3000) {
      QueueResponse("UPG,ERR,range\n");
      return;
    }

    mkdir(upgrade::kUpgradeDir, 0755);
    CloseUpgradeFile();
    std::string staged = std::string(upgrade::kUpgradeDir) + "/staging.bin";
    int fd = open(staged.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0644);
    if (fd < 0) {
      QueueResponse("UPG,ERR,open\n");
      return;
    }
    if (ftruncate(fd, (off_t)size) != 0) {
      close(fd);
      unlink(staged.c_str());
      QueueResponse("UPG,ERR,ftruncate\n");
      return;
    }
    upgFd_ = fd;
    upgSize_ = size;
    upgChunk_ = chunk;
    upgMd5_ = md5;
    upgSeqTotal_ = (size + chunk - 1) / chunk;
    upgSeen_.assign((size_t)((upgSeqTotal_ + 7) / 8), 0);
    upgSeenCount_ = 0;
    MAICRO_INFO("upgrade: begin size=%llu md5=%s chunk=%llu seq=%llu",
                (unsigned long long)size, md5.c_str(),
                (unsigned long long)chunk, (unsigned long long)upgSeqTotal_);
    QueueResponse("UPG,B,OK\n");
    return;
  }

  // 分片: UPG,DATA,<seq>,<base64> -> UPG,D,<seq>
  if (verb == "DATA") {
    if (upgFd_ < 0) {
      QueueResponse("UPG,ERR,nobegin\n");
      return;
    }
    if (f.size() < 4) {
      QueueResponse("UPG,ERR,args\n");
      return;
    }
    uint64_t seq = strtoull(f[2].c_str(), nullptr, 10);
    if (seq >= upgSeqTotal_) {
      QueueResponse("UPG,ERR,seq\n");
      return;
    }
    std::vector<uint8_t> bin;
    if (!B64Decode(f[3], &bin)) {
      QueueResponse("UPG,ERR,b64\n");
      return;
    }
    uint64_t off = seq * upgChunk_;
    if (off + bin.size() > upgSize_) {
      QueueResponse("UPG,ERR,len\n");
      return;
    }
    ssize_t w = pwrite(upgFd_, bin.data(), bin.size(), (off_t)off);
    if (w != (ssize_t)bin.size()) {
      QueueResponse("UPG,ERR,io\n");
      return;
    }
    size_t byteIdx = (size_t)(seq / 8);
    uint8_t mask = (uint8_t)(1u << (seq % 8));
    if ((upgSeen_[byteIdx] & mask) == 0) {
      upgSeen_[byteIdx] |= mask;
      ++upgSeenCount_;
    }
    QueueResponse("UPG,D," + std::to_string(seq) + "\n");
    return;
  }

  // 结束: 校验完整文件 MD5, 通过后落成待应用状态(重启生效)
  if (verb == "END") {
    if (upgFd_ < 0) {
      QueueResponse("UPG,ERR,nobegin\n");
      return;
    }
    CloseUpgradeFile();
    std::string staged = std::string(upgrade::kUpgradeDir) + "/staging.bin";
    std::string err = upgrade::CommitPending(staged, upgMd5_);
    bool complete = (upgSeenCount_ == upgSeqTotal_);
    MAICRO_INFO("upgrade: end seq=%llu/%llu err=%s",
                (unsigned long long)upgSeenCount_,
                (unsigned long long)upgSeqTotal_,
                err.empty() ? "none" : err.c_str());
    if (err.empty()) {
      QueueResponse("UPG,E,OK\n");
    } else {
      if (!complete) {
        QueueResponse("UPG,E,ERR,incomplete\n");
      } else {
        QueueResponse("UPG,E,ERR," + err + "\n");
      }
    }
    upgSeen_.clear();
    upgSeenCount_ = 0;
    upgSize_ = 0;
    upgChunk_ = 0;
    upgSeqTotal_ = 0;
    upgMd5_.clear();
    return;
  }

  if (verb == "ABORT") {
    CloseUpgradeFile();
    unlink((std::string(upgrade::kUpgradeDir) + "/staging.bin").c_str());
    upgSeen_.clear();
    upgSeenCount_ = 0;
    upgSize_ = 0;
    upgChunk_ = 0;
    upgSeqTotal_ = 0;
    upgMd5_.clear();
    QueueResponse("UPG,X,OK\n");
    return;
  }

  QueueResponse("UPG,ERR,verb\n");
}
} // namespace maicro