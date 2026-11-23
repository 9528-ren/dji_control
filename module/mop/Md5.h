#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace maicro {
namespace upgrade {

/** 计算一段内存的 MD5, 返回 32 位小写十六进制串 */
std::string Md5Hex(const void *data, size_t len);

/** 计算文件的 MD5(流式读取, 不整体载入内存); 失败返回 false */
bool Md5File(const std::string &path, std::string *out);

/** 当前可执行文件的绝对路径(/proc/self/exe); 失败返回空串 */
std::string SelfExePath();

} // namespace upgrade
} // namespace maicro
