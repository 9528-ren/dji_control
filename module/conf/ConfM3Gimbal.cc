#include "ConfM3Gimbal.h"

#include <stdlib.h>

#include "MaicroLogger.inl"

namespace maicro {
int ConfM3Gimbal::ParseConfFile(const std::string &filePath) {
  int ret = 0;
  ret = cfg_.parseFile(filePath.c_str());
  if (ret) {
    MAICRO_ERROR("load config file error, file: %s, err: %s", filePath.c_str(),
                 cfg_.getErrorString(ret));
    return -1;
  }

  // 应用信息
  userAppName = GetStringPara("User Info", "user_app_name", "");
  userAppID = GetStringPara("User Info", "user_app_id", "");
  userAppKey = GetStringPara("User Info", "user_app_key", "");
  userAppLicense = GetStringPara("User Info", "user_app_license", "");
  userDeveloperAccount = GetStringPara("User Info", "user_dev_account", "");
  userBaudrate = GetStringPara("User Info", "user_baudrate", "460800");

  // 设备信息
  productAlias = GetStringPara("Dev Info", "product_alias", "M3_GIMBAL_BTN");
  serialNumber = GetStringPara("Dev Info", "serial_number", "M3WGTGIMBAL0001");

  // 云台配置
  gimbalMountPos = GetIntPara("Gimbal Info", "gimbal_mount_pos", 1);
  pitchStep = GetFloatPara("Gimbal Info", "pitch_step", 10);
  rotateTime = GetFloatPara("Gimbal Info", "rotate_time", 1);

  // 飞行配置. 经纬度用 double 解析, float 精度只有米级
  targetLatitude = GetDoublePara("Flight Info", "target_latitude", 0);
  targetLongitude = GetDoublePara("Flight Info", "target_longitude", 0);
  targetHeight = GetFloatPara("Flight Info", "target_height", 30);
  minEnrouteHeight = GetFloatPara("Flight Info", "min_enroute_height", 30);
  flySpeed = GetIntPara("Flight Info", "fly_speed", 5);
  maxFlyDistance = GetFloatPara("Flight Info", "max_fly_distance", 500);
  moveDistance = GetFloatPara("Flight Info", "move_distance", 5);
  autoConfirmLanding =
      GetBoolPara("Flight Info", "auto_confirm_landing", false);

  // 控件配置
  widgetCfgDir = GetStringPara("Widget Info", "widget_cfg_dir", "widget_cfg");

  // 航迹配置. 全部带默认值: [Track Info] 段缺失时用默认值继续启动,
  // 只打 ERROR 日志(ConfBase 既有行为), 不让服务起不来。
  trackDir = GetStringPara("Track Info", "track_dir", "track");
  trackMinSatellites = GetIntPara("Track Info", "track_min_satellites", 10);
  trackPointTimeoutS = GetIntPara("Track Info", "track_point_timeout_s", 120);
  trackArriveTolerance =
      GetFloatPara("Track Info", "track_arrive_tolerance", 2.0f);
  trackGimbalSettleMs = GetIntPara("Track Info", "track_gimbal_settle_ms", 3000);
  trackAltCrossCheckM =
      GetFloatPara("Track Info", "track_alt_crosscheck_m", 10.0f);
  trackMaxPoints = GetIntPara("Track Info", "track_max_points", 200);
  trackRequireNetworkRtk =
      GetBoolPara("Track Info", "track_require_network_rtk", false);
  trackStopOnRtkLoss =
      GetBoolPara("Track Info", "track_stop_on_rtk_loss", false);

  if (userAppName.empty() || userAppID.empty() || userAppKey.empty() ||
      userAppLicense.empty() || userDeveloperAccount.empty()) {
    MAICRO_ERROR("[User Info] incomplete, check %s", filePath.c_str());
    return -2;
  }
  return 0;
}

double ConfM3Gimbal::GetDoublePara(const std::string &s_type,
                                   const std::string &key_words,
                                   double init_val) {
  std::string value = cfg_.getValue(s_type, key_words);
  if (value == "") {
    MAICRO_ERROR("Parse config file error: [%s, %s]", s_type.c_str(),
                 key_words.c_str());
    return init_val;
  }
  return strtod(value.c_str(), nullptr);
}
}  // namespace maicro
