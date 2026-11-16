#ifndef HAL_NETWORK_H
#define HAL_NETWORK_H
#include "dji_platform.h"
#ifdef __cplusplus
extern "C" {
#endif

#define LINUX_NETWORK_DEV           "eth0"
#define USB_NET_ADAPTER_VID          (0x0955)
#define USB_NET_ADAPTER_PID          (0x7020)
#define LINUX_CMD_STR_MAX_SIZE      (128)

T_DjiReturnCode HalNetWork_Init(const char *ipAddr, const char *netMask, T_DjiNetworkHandle *halObj);
T_DjiReturnCode HalNetWork_DeInit(T_DjiNetworkHandle halObj);
T_DjiReturnCode HalNetWork_GetDeviceInfo(T_DjiHalNetworkDeviceInfo *deviceInfo);

#ifdef __cplusplus
}
#endif
#endif
