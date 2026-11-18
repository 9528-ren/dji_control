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

  // 航迹配置(全部有默认值, 老配置文件不加 [Track Info] 段也能正常启动)
  std::string trackDir;          // 航迹文件存储目录(相对运行目录)
  int trackMinSatellites;        // 采点/执行要求的最小可见卫星数
  int trackPointTimeoutS;        // 单点飞行基础超时(秒)
  float trackArriveTolerance;    // 到位判定容差(m), 超 3 倍则中止整条航迹
  int trackGimbalSettleMs;       // 云台转到目标俯仰的等待上限(ms)
  float trackAltCrossCheckM;     // 椭球高交叉校验容差(m)
  int trackMaxPoints;            // 单条航迹最大航点数(仅日志提示, 上限由 TrackStore 强制)
  bool trackRequireNetworkRtk;   // 是否额外要求负载网络RTK处于传输态
  bool trackStopOnRtkLoss;       // 掉解时是否用最后可信坐标强制停住

 private:
  double GetDoublePara(const std::string &s_type, const std::string &key_words,
                       double init_val);
};
}  // namespace maicro
