#include "dji_psdk_api.h"
#include <string.h>
#include <string>
#include "MaicroLogger.inl"
#include "dji_logger.h"
#include "dji_platform.h"
#include "hal/hal_usb_bulk.h"
#include "osal/osal.h"
#include "osal/osal_fs.h"
#include "osal/osal_socket.h"

namespace maicro {
T_DjiReturnCode DjiUser_PrintConsole(const uint8_t *data, uint16_t dataLen) {
  std::string msg(reinterpret_cast<const char *>(data), dataLen);
  while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r' || msg.back() == '\0')) {
    msg.pop_back();
  }
  if (msg.empty()) return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
  if (msg.find("[Error]") != std::string::npos) {
    MAICRO_ERROR("[PSDK] %s", msg.c_str());
  } else if (msg.find("[Warn]") != std::string::npos) {
    MAICRO_WARN("[PSDK] %s", msg.c_str());
  } else {
    MAICRO_INFO("[PSDK] %s", msg.c_str());
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiUser_PrepareSystemEnvironment(void) {
  T_DjiReturnCode returnCode;
  T_DjiOsalHandler osalHandler;
  memset(&osalHandler, 0, sizeof(osalHandler));
  osalHandler.TaskCreate = Osal_TaskCreate;
  osalHandler.TaskDestroy = Osal_TaskDestroy;
  osalHandler.TaskSleepMs = Osal_TaskSleepMs;
  osalHandler.MutexCreate = Osal_MutexCreate;
  osalHandler.MutexDestroy = Osal_MutexDestroy;
  osalHandler.MutexLock = Osal_MutexLock;
  osalHandler.MutexUnlock = Osal_MutexUnlock;
  osalHandler.SemaphoreCreate = Osal_SemaphoreCreate;
  osalHandler.SemaphoreDestroy = Osal_SemaphoreDestroy;
  osalHandler.SemaphoreWait = Osal_SemaphoreWait;
  osalHandler.SemaphoreTimedWait = Osal_SemaphoreTimedWait;
  osalHandler.SemaphorePost = Osal_SemaphorePost;
  osalHandler.GetTimeMs = Osal_GetTimeMs;
  osalHandler.GetTimeUs = Osal_GetTimeUs;
  osalHandler.GetRandomNum = Osal_GetRandomNum;
  osalHandler.Malloc = Osal_Malloc;
  osalHandler.Free = Osal_Free;

  T_DjiLoggerConsole printConsole;
  memset(&printConsole, 0, sizeof(printConsole));
  printConsole.func = DjiUser_PrintConsole;
  printConsole.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO;
  printConsole.isSupportColor = false;

  T_DjiHalUsbBulkHandler usbBulkHandler;
  memset(&usbBulkHandler, 0, sizeof(usbBulkHandler));
  usbBulkHandler.UsbBulkInit = HalUsbBulk_Init;
  usbBulkHandler.UsbBulkDeInit = HalUsbBulk_DeInit;
  usbBulkHandler.UsbBulkWriteData = HalUsbBulk_WriteData;
  usbBulkHandler.UsbBulkReadData = HalUsbBulk_ReadData;
  usbBulkHandler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;

  T_DjiFileSystemHandler fileSystemHandler;
  memset(&fileSystemHandler, 0, sizeof(fileSystemHandler));
  fileSystemHandler.FileOpen = Osal_FileOpen;
  fileSystemHandler.FileClose = Osal_FileClose;
  fileSystemHandler.FileWrite = Osal_FileWrite;
  fileSystemHandler.FileRead = Osal_FileRead;
  fileSystemHandler.FileSeek = Osal_FileSeek;
  fileSystemHandler.FileSync = Osal_FileSync;
  fileSystemHandler.DirOpen = Osal_DirOpen;
  fileSystemHandler.DirClose = Osal_DirClose;
  fileSystemHandler.DirRead = Osal_DirRead;
  fileSystemHandler.Mkdir = Osal_Mkdir;
  fileSystemHandler.Unlink = Osal_Unlink;
  fileSystemHandler.Rename = Osal_Rename;
  fileSystemHandler.Stat = Osal_Stat;

  T_DjiSocketHandler socketHandler;
  memset(&socketHandler, 0, sizeof(socketHandler));
  socketHandler.Socket = Osal_Socket;
  socketHandler.Bind = Osal_Bind;
  socketHandler.Close = Osal_Close;
  socketHandler.UdpSendData = Osal_UdpSendData;
  socketHandler.UdpRecvData = Osal_UdpRecvData;
  socketHandler.TcpListen = Osal_TcpListen;
  socketHandler.TcpAccept = Osal_TcpAccept;
  socketHandler.TcpConnect = Osal_TcpConnect;
  socketHandler.TcpSendData = Osal_TcpSendData;
  socketHandler.TcpRecvData = Osal_TcpRecvData;

  returnCode = DjiPlatform_RegOsalHandler(&osalHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { MAICRO_ERROR("register osal error"); return returnCode; }
  returnCode = DjiLogger_AddConsole(&printConsole);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { MAICRO_ERROR("add console error"); return returnCode; }
  returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { MAICRO_ERROR("register usb bulk error"); return returnCode; }
  returnCode = DjiPlatform_RegSocketHandler(&socketHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { MAICRO_ERROR("register socket error"); return returnCode; }
  returnCode = DjiPlatform_RegFileSystemHandler(&fileSystemHandler);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { MAICRO_ERROR("register fs error"); return returnCode; }
  MAICRO_INFO("psdk system environment ready (USB HAL)");
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiUser_FillInUserInfo(T_DjiUserInfo *userInfo, const ConfM3Gimbal *conf) {
  memset(userInfo, 0, sizeof(*userInfo));
  strncpy(userInfo->appName, conf->userAppName.c_str(), sizeof(userInfo->appName) - 1);
  strncpy(userInfo->appId, conf->userAppID.c_str(), sizeof(userInfo->appId) - 1);
  strncpy(userInfo->appKey, conf->userAppKey.c_str(), sizeof(userInfo->appKey) - 1);
  strncpy(userInfo->appLicense, conf->userAppLicense.c_str(), sizeof(userInfo->appLicense) - 1);
  strncpy(userInfo->developerAccount, conf->userDeveloperAccount.c_str(), sizeof(userInfo->developerAccount) - 1);
  strncpy(userInfo->baudRate, conf->userBaudrate.c_str(), sizeof(userInfo->baudRate) - 1);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
}  // namespace maicro
