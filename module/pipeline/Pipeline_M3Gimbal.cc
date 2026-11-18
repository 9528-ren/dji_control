#include "Pipeline_M3Gimbal.h"

#include "MaicroLogger.inl"

namespace maicro {
PipelineM3Gimbal::PipelineM3Gimbal(const std::string &cfgPath)
    : cfgPath_(cfgPath) {}

PipelineM3Gimbal::~PipelineM3Gimbal() { DeInit(); }

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
  mopCmd_ = new MopCommandServer;
  if (mopCmd_->Init(gimbal_, flight_, track_)) {
    MAICRO_ERROR("mop cmd server init error");
    // 不阻止启动, widget 仍可用
  }

  MAICRO_INFO("pipeline init ok, waiting for Pilot widget input / MOP commands");
  return 0;
}

void PipelineM3Gimbal::DeInit(void) {
  // 释放顺序: 先切断命令来源(控件/MOP 线程), 再释放被它们引用的
  // flight_/gimbal_, 否则这些线程可能在对象已 delete 后仍调用 PostCmd (悬空指针)
  if (widget_) {
    delete widget_;
    widget_ = nullptr;
  }
  if (mopCmd_) {
    mopCmd_->DeInit();
    delete mopCmd_;
    mopCmd_ = nullptr;
  }
  if (flight_) {
    flight_->DeInit();
    delete flight_;
    flight_ = nullptr;
  }
  // track_ 被 flight_ 与 mopCmd_ 引用, 必须在两者都释放之后再删,
  // 否则它们的工作线程可能访问已析构的存储(悬空指针)
  if (track_) {
    delete track_;
    track_ = nullptr;
  }
  if (gimbal_) {
    gimbal_->DeInit();
    delete gimbal_;
    gimbal_ = nullptr;
  }
  if (vehicle_) {
    vehicle_->DeInitVehicleObject();
    delete vehicle_;
    vehicle_ = nullptr;
  }
  if (conf_) {
    delete conf_;
    conf_ = nullptr;
  }
}

int64_t PipelineM3Gimbal::InitConf(void) {
  conf_ = new ConfM3Gimbal;
  if (conf_->ParseConfFile(cfgPath_)) {
    return -1;
  }
  MAICRO_INFO("load conf ok: %s", cfgPath_.c_str());
  MAICRO_INFO("  app name: %s, app id: %s", conf_->userAppName.c_str(),
              conf_->userAppID.c_str());
  MAICRO_INFO("  widget cfg dir: %s", conf_->widgetCfgDir.c_str());
  return 0;
}

int64_t PipelineM3Gimbal::InitVehicle(void) {
  vehicle_ = new VehicleDjiPsdk;
  return vehicle_->InitVehicleObject(conf_);
}

int64_t PipelineM3Gimbal::InitGimbal(void) {
  gimbal_ = new GimbalController;
  return gimbal_->Init(vehicle_, (E_MaicroMountPosition)conf_->gimbalMountPos,
                       conf_->pitchStep, conf_->rotateTime);
}

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
  track_ = new TrackStore;
  if (track_->Init(conf_->trackDir) != 0) {
    MAICRO_ERROR("track store init failed (dir=%s), track feature disabled",
                 conf_->trackDir.c_str());
  }

  flight_ = new FlightController;
  int64_t ret = flight_->Init(vehicle_, flightConf);
  // 注入航迹存储与主云台挂载位置(航迹回放转云台用; mountPos 是 VehicleBase
  // 的 protected 成员, FlightController 无法经指针访问, 故在此显式注入)
  flight_->SetTrackStore(track_);
  flight_->SetGimbalMountPos((E_MaicroMountPosition)conf_->gimbalMountPos);
  return ret;
}

int64_t PipelineM3Gimbal::InitWidget(void) {
  widget_ = new WidgetHandler;
  return widget_->Init(conf_->widgetCfgDir, gimbal_, flight_);
}
}  // namespace maicro
