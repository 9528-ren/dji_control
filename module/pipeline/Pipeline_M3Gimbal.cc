#include "Pipeline_M3Gimbal.h"

#include "MaicroLogger.inl"

namespace maicro {
PipelineM3Gimbal::PipelineM3Gimbal(const std::string &cfgPath)
    : cfgPath_(cfgPath) {}

PipelineM3Gimbal::~PipelineM3Gimbal() { DeInit(); }

/**
 * 按依赖顺序装配整条链路: 配置 -> 硬件 -> 云台 -> 飞控(含航迹存储) ->
 * 控件 -> 启动 PSDK 应用 -> MOP 服务端。
 *
 * 顺序不可调换: 控件回调会投递命令, 必须在云台与飞控就绪后注册;
 * PSDK 应用必须在所有模块(含控件)注册完成后才能启动。
 * MOP 初始化失败不阻止启动 —— Pilot 控件仍可用。
 */
int64_t PipelineM3Gimbal::Init(void) {
  if (InitConf()) {
    MAICRO_ERROR("init conf error");
    return -1;
  }
  if (InitVehicle()) {
    MAICRO_ERROR("init vehicle error");
    return -2;
  }
  if (InitGimbal()) {
    MAICRO_ERROR("init gimbal error");
    return -3;
  }
  if (InitFlight()) {
    MAICRO_ERROR("init flight error");
    return -4;
  }
  // 控件回调会投递云台/飞行命令, 必须在两者就绪之后注册
  if (InitWidget()) {
    MAICRO_ERROR("init widget error");
    return -5;
  }

  if (vehicle_->StartApplication()) {
    MAICRO_ERROR("start application error");
    return -6;
  }
  // 初始化 MOP 命令服务端 (接收 App 端指令)
  mopCmd_ = std::make_unique<MopCommandServer>();
  if (mopCmd_->Init(gimbal_.get(), flight_.get(), track_.get())) {
    MAICRO_ERROR("mop cmd server init error");
    // 不阻止启动, widget 仍可用
  }

  MAICRO_INFO(
      "pipeline init ok, waiting for Pilot widget input / MOP commands");
  return 0;
}

/**
 * 释放整条链路。内存由 unique_ptr 按声明逆序自动回收, 本函数只负责
 * **必须在析构前显式完成**的事: 停掉会回调进这些对象的线程。
 * 顺序: 切断命令来源(控件/MOP) -> 停飞控与云台执行线程 -> 释放航迹存储
 * -> 反初始化硬件。详见函数体内的分步说明与头文件的声明顺序注释。
 */
void PipelineM3Gimbal::DeInit(void) {
  // 内存释放已由 unique_ptr 按声明逆序自动完成(见头文件注释)。
  // 这里只做**必须在析构前显式完成**的事: 停掉会回调进这些对象的线程。
  // 顺序仍是先切断命令来源, 再停被引用的执行器 —— 否则工作线程可能在
  // 对象析构后仍调用 PostCmd(悬空指针)。
  if (widget_) {
    // WidgetHandler 无独立线程, 其回调由 SDK 收发线程触发;
    // 置空即可断开后续投递(实际注销随 PSDK 反初始化完成)。
    widget_.reset();
  }
  if (mopCmd_) {
    mopCmd_->DeInit();   // 停 accept/收包/发送线程
    mopCmd_.reset();
  }
  if (flight_) {
    flight_->DeInit();   // 停飞行执行线程
    flight_.reset();
  }
  if (gimbal_) {
    gimbal_->DeInit();   // 停云台执行线程
    gimbal_.reset();
  }
  // track_ 被 flight_ 与 mopCmd_ 引用, 此刻两者均已释放, 可安全销毁
  track_.reset();
  if (vehicle_) {
    vehicle_->DeInitVehicleObject();
    vehicle_.reset();
  }
  conf_.reset();
}

/// 解析配置文件到 ConfM3Gimbal。失败即中止启动(凭据缺失无法注册 PSDK)。
int64_t PipelineM3Gimbal::InitConf(void) {
  conf_ = std::make_unique<ConfM3Gimbal>();
  if (conf_->ParseConfFile(cfgPath_)) {
    return -1;
  }
  MAICRO_INFO("load conf ok: %s", cfgPath_.c_str());
  MAICRO_INFO("  app name: %s, app id: %s", conf_->userAppName.c_str(),
              conf_->userAppID.c_str());
  MAICRO_INFO("  widget cfg dir: %s", conf_->widgetCfgDir.c_str());
  return 0;
}

/// 创建并初始化硬件抽象层(PSDK 环境/核心/云台/订阅/飞控)。
int64_t PipelineM3Gimbal::InitVehicle(void) {
  vehicle_ = std::make_unique<VehicleDjiPsdk>();
  return vehicle_->InitVehicleObject(conf_.get());
}

/**
 * 创建云台控制器。挂载口取**硬件层的探测结果**而非配置原值:
 * 配置为 0 表示自动探测, 该原值对云台操作没有意义。
 */
int64_t PipelineM3Gimbal::InitGimbal(void) {
  gimbal_ = std::make_unique<GimbalController>();
  // 挂载口取自 vehicle_ 的探测结果而非直接照抄 conf_->gimbalMountPos:
  // 配置为 0 时代表"自动探测", 该原始值对云台控制器没有意义
  return gimbal_->Init(vehicle_.get(), vehicle_->QueryGimbalMountPos(),
                       conf_->pitchStep, conf_->rotateTime);
}

/**
 * 创建航迹存储与飞控, 并把配置映射为 S_FlightConf。
 *
 * 存储先于飞控创建: 飞控与 MOP 服务端都持有它的指针。
 * 存储初始化失败(目录不可建)不阻止启动 —— 按键仍可用, 航迹命令回 ERROR:io,
 * 不静默丢数据。云台挂载口同样注入探测结果。
 */
int64_t PipelineM3Gimbal::InitFlight(void) {
  S_FlightConf flightConf;
  flightConf.latitude = conf_->targetLatitude;
  flightConf.longitude = conf_->targetLongitude;
  flightConf.height = conf_->targetHeight;
  flightConf.minEnrouteHeight = conf_->minEnrouteHeight;
  flightConf.speed = (uint8_t)conf_->flySpeed;
  flightConf.maxDistance = conf_->maxFlyDistance;
  flightConf.moveDistance = conf_->moveDistance;
  flightConf.autoConfirmLanding = conf_->autoConfirmLanding;

  // 航迹执行参数
  flightConf.trackMinSatellites = (uint16_t)conf_->trackMinSatellites;
  flightConf.trackPointTimeoutS = (uint32_t)conf_->trackPointTimeoutS;
  flightConf.trackArriveTolerance = conf_->trackArriveTolerance;
  flightConf.trackGimbalSettleMs = (uint32_t)conf_->trackGimbalSettleMs;
  flightConf.trackAltCrossCheckM = conf_->trackAltCrossCheckM;
  flightConf.trackRequireNetworkRtk = conf_->trackRequireNetworkRtk;
  flightConf.trackStopOnRtkLoss = conf_->trackStopOnRtkLoss;

  // 航迹存储: 先于 flight_ 创建, 因为 flight_ 与 mopCmd_ 都持有它的指针。
  // Init 失败(目录不可建)不阻止服务启动 —— 云台/飞行按键仍可用,
  // 只是航迹命令会回 ERROR:io, 不静默丢数据。
  track_ = std::make_unique<TrackStore>();
  if (track_->Init(conf_->trackDir) != 0) {
    MAICRO_ERROR("track store init failed (dir=%s), track feature disabled",
                 conf_->trackDir.c_str());
  }

  flight_ = std::make_unique<FlightController>();
  int64_t ret = flight_->Init(vehicle_.get(), flightConf);
  // 注入航迹存储与主云台挂载位置(航迹回放转云台用; mountPos 是 VehicleBase
  // 的 protected 成员, FlightController 无法经指针访问, 故在此显式注入)。
  // 注入探测结果而非配置原值: 配置为 0 表示自动探测, 原值不可直接使用
  flight_->SetTrackStore(track_.get());
  flight_->SetGimbalMountPos(vehicle_->QueryGimbalMountPos());
  return ret;
}

/// 创建控件处理器并注册按钮回调。必须在云台与飞控就绪之后调用。
int64_t PipelineM3Gimbal::InitWidget(void) {
  widget_ = std::make_unique<WidgetHandler>();
  return widget_->Init(conf_->widgetCfgDir, gimbal_.get(), flight_.get());
}
} // namespace maicro