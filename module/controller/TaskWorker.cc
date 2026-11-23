#include "TaskWorker.h"

#include <chrono>

#include "MaicroLogger.inl"

namespace maicro {
TaskWorker::TaskWorker() {}

TaskWorker::~TaskWorker() { Stop(); }

/**
 * 启动执行线程。name 仅用于日志标识(如 "flight"/"gimbal")。
 * handler 在本线程内被调用, 因此可以执行阻塞的 PSDK 接口 ——
 * 这正是把回调与执行分离的目的: SDK 收发线程里不能阻塞。
 */
void TaskWorker::Start(const std::string &name, Handler handler) {
  name_ = name;
  handler_ = handler;
  running_ = true;
  thread_ = std::thread(&TaskWorker::Loop, this);
}

/**
 * 停止执行线程并等待其退出。清空未取走的命令, 唤醒所有等待者。
 * 幂等: 未启动或已停止时直接返回。析构会自动调用。
 */
void TaskWorker::Stop(void) {
  {
    std::lock_guard<std::mutex> lck(mutex_);
    if (!running_) return;
    running_ = false;
    pendingCmd_ = 0;
  }
  cond_.notify_all();
  if (thread_.joinable()) thread_.join();
}

/**
 * 投递命令(线程安全, 非阻塞)。**单槽语义**: 若已有未取走的命令,
 * 直接覆盖它并返回被覆盖的旧命令号(0 表示原本没有)。
 * 这样"飞行途中按降落"能立即生效, 而不是排队等前一个动作做完。
 * token 用于让执行中的长任务识别自己是否已过期(见 ExecutingToken)。
 */
int TaskWorker::Post(int cmd, uint64_t token) {
  int oldCmd;
  {
    std::lock_guard<std::mutex> lck(mutex_);
    oldCmd = pendingCmd_;
    pendingCmd_ = cmd;
    pendingToken_ = token;
  }
  cond_.notify_all();
  return oldCmd;
}

/**
 * 仅在完全空闲时投递(线程安全, 非阻塞)。
 * 与 Post 的区别: 有待执行命令或正在执行时**拒绝**而不是覆盖, 返回 false。
 * 供不希望抢占正在进行的动作的调用方使用(如非 latest 模式的语音脚本)。
 */
bool TaskWorker::PostIfIdle(int cmd, uint64_t token) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || pendingCmd_ != 0 || executing_) return false;
    pendingCmd_ = cmd;
    pendingToken_ = token;
  }
  cond_.notify_one();
  return true;
}

/**
 * 取当前正在执行的命令的 token。执行中的长任务据此判断自己是否已被
 * 新命令取代: 若不等则应立即收尾退出, 避免过期任务继续下发控制。
 */
uint64_t TaskWorker::ExecutingToken() {
  std::lock_guard<std::mutex> lock(mutex_);
  return executingToken_;
}

/**
 * 是否有待执行的新命令, 或执行器已停止。
 * 执行中的长任务每轮循环都应检查它, 以便被新指令及时打断。
 * 停止时也返回 true, 使循环能在退出前正常收尾。
 */
bool TaskWorker::HasPending(void) {
  std::lock_guard<std::mutex> lck(mutex_);
  return pendingCmd_ != 0 || !running_;
}

/**
 * 可中断睡眠。睡满 ms 返回 true; 期间来了新命令或执行器停止则提前
 * 返回 false。长任务用它替代裸 sleep, 保证抢占延迟不超过一个循环周期。
 */
bool TaskWorker::SleepMs(uint32_t ms) {
  std::unique_lock<std::mutex> lck(mutex_);
  return !cond_.wait_for(lck, std::chrono::milliseconds(ms),
                         [this] { return pendingCmd_ != 0 || !running_; });
}

/**
 * 执行线程主循环: 等命令 -> 取命令并记录 token -> 调 handler -> 标记空闲。
 * 取命令与调 handler 之间不持锁, 因此 handler 执行期间仍可接收新命令,
 * 这是"边飞边接受抢占"的基础。
 */
void TaskWorker::Loop(void) {
  MAICRO_INFO("%s worker thread start", name_.c_str());
  while (true) {
    int cmd;
    {
      std::unique_lock<std::mutex> lck(mutex_);
      cond_.wait(lck, [this] { return !running_ || pendingCmd_ != 0; });
      if (!running_) break;
      cmd = pendingCmd_;
      executingToken_ = pendingToken_;
      pendingCmd_ = 0;
      executing_ = true;
    }
    handler_(cmd);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      executing_ = false;
    }
  }
  MAICRO_INFO("%s worker thread exit", name_.c_str());
}
}  // namespace maicro
