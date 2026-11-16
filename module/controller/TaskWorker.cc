#include "TaskWorker.h"

#include <chrono>

#include "MaicroLogger.inl"

namespace maicro {
TaskWorker::TaskWorker() {}

TaskWorker::~TaskWorker() { Stop(); }

void TaskWorker::Start(const std::string &name, Handler handler) {
  name_ = name;
  handler_ = handler;
  running_ = true;
  thread_ = std::thread(&TaskWorker::Loop, this);
}

void TaskWorker::Stop(void) {
  {
    std::lock_guard<std::mutex> lck(mutex_);
    if (!running_) return;
    running_ = false;
  }
  cond_.notify_all();
  if (thread_.joinable()) thread_.join();
}

int TaskWorker::Post(int cmd) {
  int oldCmd;
  {
    std::lock_guard<std::mutex> lck(mutex_);
    oldCmd = pendingCmd_;
    pendingCmd_ = cmd;
  }
  cond_.notify_all();
  return oldCmd;
}

bool TaskWorker::HasPending(void) {
  std::lock_guard<std::mutex> lck(mutex_);
  return pendingCmd_ != 0 || !running_;
}

bool TaskWorker::SleepMs(uint32_t ms) {
  std::unique_lock<std::mutex> lck(mutex_);
  return !cond_.wait_for(lck, std::chrono::milliseconds(ms),
                         [this] { return pendingCmd_ != 0 || !running_; });
}

void TaskWorker::Loop(void) {
  MAICRO_INFO("%s worker thread start", name_.c_str());
  while (true) {
    int cmd;
    {
      std::unique_lock<std::mutex> lck(mutex_);
      cond_.wait(lck, [this] { return !running_ || pendingCmd_ != 0; });
      if (!running_) break;
      cmd = pendingCmd_;
      pendingCmd_ = 0;
    }
    handler_(cmd);
  }
  MAICRO_INFO("%s worker thread exit", name_.c_str());
}
}  // namespace maicro
