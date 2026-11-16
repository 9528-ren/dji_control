#pragma once
#include <stdint.h>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace maicro {
// 单线程命令执行器.
// PSDK 控件回调运行在 SDK 收发线程里, 回调中不能执行阻塞的 PSDK 接口,
// 所以回调只 Post 命令, 由本线程执行.
// 单槽: 未执行的旧命令会被新命令覆盖, 避免积压.
class TaskWorker {
 public:
  typedef std::function<void(int cmd)> Handler;

  TaskWorker();
  ~TaskWorker();

  void Start(const std::string &name, Handler handler);
  void Stop(void);

  // 投递命令(线程安全, 非阻塞), 返回被覆盖的旧命令, 0 表示没有
  int Post(int cmd);

  // 是否有待执行的新命令(执行中的长任务可据此中断)
  bool HasPending(void);

  // 可中断睡眠: 有新命令或停止时提前返回 false
  bool SleepMs(uint32_t ms);

 private:
  void Loop(void);

 private:
  std::string name_;
  Handler handler_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cond_;
  int pendingCmd_ = 0;
  bool running_ = false;
};
}  // namespace maicro
