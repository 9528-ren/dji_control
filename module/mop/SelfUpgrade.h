#pragma once

#include <string>

namespace maicro {
namespace upgrade {

/** 升级暂存目录, 与 App 侧协议约定一致 */
extern const char *const kUpgradeDir;

/**
 * 启动时检查是否有已上传完成的新程序并应用。
 *
 * 调用时机: main() 里日志初始化之后、任何业务初始化之前。
 * - 只有 pending.bin 的 MD5 与 pending.md5 一致才替换, 避免半截文件把程序弄坏
 * - 替换前把当前程序备份为 <exe>.bak.<时间戳>
 * - 替换成功后 execv 新程序原地接管(带 M3_UPGRADE_APPLIED 环境变量防死循环)
 * - 没有待升级文件时立即返回, 不做任何 IO
 */
void ApplyPendingAndExec(int argc, char *argv[]);

/**
 * 把暂存文件落成"待应用"状态(先校验再改名), 供 MOP 升级命令在 UPG,END 时调用。
 * 成功时写 pending.bin + pending.md5, 返回空串; 失败返回错误原因。
 */
std::string CommitPending(const std::string &stagedFile,
                          const std::string &expectedMd5);

/** 丢弃暂存/待应用文件(MD5 不符或用户取消时调用) */
void DiscardPending();

} // namespace upgrade
} // namespace maicro
