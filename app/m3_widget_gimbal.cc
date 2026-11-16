#include <signal.h>
#include <sys/resource.h>
#include <unistd.h>

#include <condition_variable>
#include <mutex>
#include <string>

#include "MaicroLogger.inl"
#include "pipeline/Pipeline_M3Gimbal.h"
#include "version.h"

using namespace maicro;

static void raiseFdLimit() {
  struct rlimit rl;
  if (getrlimit(RLIMIT_NOFILE, &rl) == 0) {
    int old_cur = rl.rlim_cur;
    rl.rlim_cur = rl.rlim_max = 8192;
    if (setrlimit(RLIMIT_NOFILE, &rl) == 0) {
      MAICRO_INFO("fd limit: %d -> %d", old_cur, (int)rl.rlim_cur);
    } else {
      MAICRO_ERROR("fd limit setrlimit failed errno=%d", errno);
    }
  }
}

static bool s_finishFlag = false;
static std::mutex s_finishMutex;
static std::condition_variable s_finishedCond;

static void handleSig(int singal) {
  if (SIGINT == singal || SIGTERM == singal) {
    std::unique_lock<std::mutex> lck(s_finishMutex);
    s_finishFlag = true;
    s_finishedCond.notify_one();
  }
}

static void initLog(const std::string &logFile) {
  maicro::Logger::instance().init("m3_widget_gimbal", logFile.c_str());
}

// 用法: m3_widget_gimbal [配置文件] [日志文件]
int main(int argc, char *argv[]) {
  std::string cfgPath = argc > 1 ? argv[1] : "cfg/m3_gimbal.cfg";
  std::string logFile = argc > 2 ? argv[2] : "log/m3_widget_gimbal.log";

  initLog(logFile);
  raiseFdLimit();

  signal(SIGINT, handleSig);
  signal(SIGTERM, handleSig);
  // 初始化日志
  initLog(logFile);
  MAICRO_INFO("========================================================");
  MAICRO_INFO("M3_Widget_Gimbal task start");
  MAICRO_INFO("  Version: %s", M3_GIMBAL_VERSION);
  MAICRO_INFO("  Config: %s", cfgPath.c_str());
  MAICRO_INFO("  Log: %s", logFile.c_str());
  MAICRO_INFO("========================================================");

  PipelineM3Gimbal *pipeline = new PipelineM3Gimbal(cfgPath);
  if (pipeline->Init()) {
    usleep(5 * 1000 * 100);
    MAICRO_ERROR("pipeline init error, shutdown and retry");
    return -1;
  }

  // 等待信号
  {
    std::unique_lock<std::mutex> lck(s_finishMutex);
    while (!s_finishFlag)
      s_finishedCond.wait(lck);
  }

  MAICRO_INFO("recv exit signal, deinit");
  pipeline->DeInit();
  delete pipeline;
  MAICRO_INFO("Game Over");
  return 0;
}