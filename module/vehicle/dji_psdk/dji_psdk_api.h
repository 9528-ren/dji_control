#pragma once
#include "conf/ConfM3Gimbal.h"
#include "dji_core.h"
#include "dji_typedef.h"

namespace maicro {
// PSDK内部日志输出, 转到 MAICRO 日志
T_DjiReturnCode DjiUser_PrintConsole(const uint8_t *data, uint16_t dataLen);

// 注册 OSAL / USB Bulk / 文件系统 / Socket 等平台接口(妙算3)
T_DjiReturnCode DjiUser_PrepareSystemEnvironment(void);

// 用配置文件填充 PSDK 用户信息
T_DjiReturnCode DjiUser_FillInUserInfo(T_DjiUserInfo *userInfo,
                                       const ConfM3Gimbal *conf);
}  // namespace maicro
