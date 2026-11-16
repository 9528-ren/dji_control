#pragma once
#include "ConfBase.h"

namespace maicro {
class ConfM3Gimbal : public ConfBase {
 public:
  virtual int ParseConfFile(const std::string &filePath);

 public:
  // 应用信息
  std::string userAppName;
  std::string userAppID;
  std::string userAppKey;
  std::string userAppLicense;
  std::string userDeveloperAccount;
  std::string userBaudrate;

  // 设备信息
  std::string productAlias;
  std::string serialNumber;

  // 云台配置
  int gimbalMountPos;
  float pitchStep;
  float rotateTime;

  // 飞行配置: 固定目标点
  double targetLatitude;    // 度
  double targetLongitude;   // 度
  float targetHeight;       // 相对起飞点高度, m
  float minEnrouteHeight;   // 最低航线高度(相对起飞点), m
  int flySpeed;             // m/s
  float maxFlyDistance;     // 目标点离当前位置的最大允许距离, m
  float moveDistance;       // 方向按键每次平移距离, m
  bool autoConfirmLanding;  // 降落保护触发时自动确认降落

  // 控件配置
  std::string widgetCfgDir;

 private:
  double GetDoublePara(const std::string &s_type, const std::string &key_words,
                       double init_val);
};
}  // namespace maicro
