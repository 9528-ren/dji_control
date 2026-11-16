#pragma once
#include <stdio.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include "spdlog/sinks/rotating_file_sink.h"
#include <spdlog/fmt/bundled/printf.h>


#include <iostream>
#include <memory>


enum MAICRO_ERR_LEVEL {
  MAICRO_ERR_TRACE = 0x0,
  MAICRO_ERR_DEBUG = 0x1,
  MAICRO_ERR_INFO = 0x2,
  MAICRO_ERR_WARN = 0x3,
  MAICRO_ERR_ERROR = 0x4,
  MAICRO_ERR_CRITICAL = 0x5,
  MAICRO_ERR_NUMBER
};

namespace maicro {

class Logger {
 public:
  static Logger &instance() {
    static Logger maicro_logger;
    return maicro_logger;
  }

  int init(const std::string &tag, const std::string &file_name) {
    if (logger_ != nullptr) {
      return -1;
    }

    setlocale(LC_ALL, "zh_CN.utf8");

    try {
      auto max_size = 1048576 * 20;
      auto max_files = 10;
      auto console_sink =
          std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::trace);
      console_sink->set_pattern("[%L][%Y/%m/%d %T.%e][%t][%s,%!:%#]: %v");

      auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          file_name, max_size, max_files);
      file_sink->set_level(spdlog::level::info);
      file_sink->set_pattern("[%L][%Y/%m/%d %T.%e][%t][%s,%!:%#]: %v");

      logger_.reset(new spdlog::logger(tag, {console_sink, file_sink}));
      logger_->set_level(spdlog::level::info);
      logger_->flush_on(spdlog::level::info);
      spdlog::flush_every(std::chrono::seconds(3));
    } catch (const spdlog::spdlog_ex &ex) {
      std::cout << "Log initialization failed: " << ex.what() << std::endl;
      return -2;
    }
    return 0;
  }

  std::unique_ptr<spdlog::logger> &logger() { return logger_; }

  Logger(const Logger &) = delete;

  Logger &operator=(const Logger &) = delete;

 private:
  std::unique_ptr<spdlog::logger> logger_ = nullptr;

  Logger() = default;

  ~Logger() = default;
};

}  // namespace maicro

template <class loggerPtr, class... Args>
void log_printf(loggerPtr &logger_ptr, spdlog::level::level_enum level_enum,
                spdlog::source_loc loc, const char *fmt,
                const Args &... args) noexcept {
  if (logger_ptr && logger_ptr->should_log(level_enum)) {
    logger_ptr->log(loc, level_enum, "{}", fmt::sprintf(fmt, args...));
  }
}

#define MAICRO_TRACE(fmt, ...)                                             \
  log_printf(maicro::Logger::instance().logger(), spdlog::level::trace,    \
             spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, fmt, \
             ##__VA_ARGS__)
#define MAICRO_DEBUG(fmt, ...)                                             \
  log_printf(maicro::Logger::instance().logger(), spdlog::level::debug,    \
             spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, fmt, \
             ##__VA_ARGS__)
#define MAICRO_INFO(fmt, ...)                                              \
  log_printf(maicro::Logger::instance().logger(), spdlog::level::info,     \
             spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, fmt, \
             ##__VA_ARGS__)
#define MAICRO_WARN(fmt, ...)                                              \
  log_printf(maicro::Logger::instance().logger(), spdlog::level::warn,     \
             spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, fmt, \
             ##__VA_ARGS__)
#define MAICRO_ERROR(fmt, ...)                                             \
  log_printf(maicro::Logger::instance().logger(), spdlog::level::err,      \
             spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, fmt, \
             ##__VA_ARGS__)
#define MAICRO_CRITICAL(fmt, ...)                                          \
  log_printf(maicro::Logger::instance().logger(), spdlog::level::critical, \
             spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, fmt, \
             ##__VA_ARGS__)

#define MAICRO_LOG(Level, Msg, ...)          \
  do {                                       \
    switch (Level) {                         \
      case MAICRO_ERR_TRACE:                 \
        MAICRO_TRACE(Msg, ##__VA_ARGS__);    \
        break;                               \
      case MAICRO_ERR_DEBUG:                 \
        MAICRO_DEBUG(Msg, ##__VA_ARGS__);    \
        break;                               \
      case MAICRO_ERR_INFO:                  \
        MAICRO_INFO(Msg, ##__VA_ARGS__);     \
        break;                               \
      case MAICRO_ERR_WARN:                  \
        MAICRO_WARN(Msg, ##__VA_ARGS__);     \
        break;                               \
      case MAICRO_ERR_ERROR:                 \
        MAICRO_ERROR(Msg, ##__VA_ARGS__);    \
        break;                               \
      case MAICRO_ERR_CRITICAL:              \
        MAICRO_CRITICAL(Msg, ##__VA_ARGS__); \
        break;                               \
      default:                               \
        break;                               \
    }                                        \
  } while (0)

#define MAICRO_CHECK_EXPR_GOTO(Expr, Label, Level, Msg, ...) \
  do {                                                       \
    if (Expr) {                                              \
      MAICRO_LOG(Level, Msg, ##__VA_ARGS__);                 \
      goto Label;                                            \
    }                                                        \
  } while (0)

#define MAICRO_CHECK_EXPR_RET(Expr, Ret, Level, Msg, ...) \
  do {                                                    \
    if (Expr) {                                           \
      MAICRO_LOG(Level, Msg, ##__VA_ARGS__);              \
      return Ret;                                         \
    }                                                     \
  } while (0)

#include <assert.h>
#define MAICRO_ASSERT(expr, msg) \
  if (!(expr)) {                 \
    MAICRO_ERROR("%s\n", msg);   \
    assert(0);                   \
  }

#define MAICRO_CHECK_RET(expr, msg, ret) \
  if (!(expr)) {                         \
    MAICRO_ERROR("%s\n", msg);           \
    return ret;                          \
  }
