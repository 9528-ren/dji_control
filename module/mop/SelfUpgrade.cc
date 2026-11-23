#include "mop/SelfUpgrade.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include "MaicroLogger.inl"
#include "mop/Md5.h"

namespace maicro {
namespace upgrade {

const char *const kUpgradeDir = "/home/dji/.m3_upgrade";

namespace {

std::string PendingBin() { return std::string(kUpgradeDir) + "/pending.bin"; }
std::string PendingMd5() { return std::string(kUpgradeDir) + "/pending.md5"; }

bool ReadMd5Marker(std::string *out) {
  FILE *fp = fopen(PendingMd5().c_str(), "rb");
  if (fp == nullptr)
    return false;
  char buf[64] = {0};
  size_t n = fread(buf, 1, sizeof(buf) - 1, fp);
  fclose(fp);
  if (n < 32)
    return false;
  std::string s(buf, 32);
  for (auto &c : s) {
    if (c >= 'A' && c <= 'F')
      c = (char)(c - 'A' + 'a');
  }
  if (out != nullptr)
    *out = s;
  return true;
}

// 复制文件(用于备份当前程序); 只在本程序自身上使用, 大小有限, 逐块复制即可
bool CopyFile(const std::string &src, const std::string &dst) {
  FILE *in = fopen(src.c_str(), "rb");
  if (in == nullptr)
    return false;
  FILE *out = fopen(dst.c_str(), "wb");
  if (out == nullptr) {
    fclose(in);
    return false;
  }
  char buf[64 * 1024];
  size_t n;
  bool ok = true;
  while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
    if (fwrite(buf, 1, n, out) != n) {
      ok = false;
      break;
    }
  }
  if (ferror(in))
    ok = false;
  fclose(in);
  fflush(out);
  fclose(out);
  if (ok)
    chmod(dst.c_str(), 0755);
  else
    unlink(dst.c_str());
  return ok;
}

} // namespace

std::string CommitPending(const std::string &stagedFile,
                          const std::string &expectedMd5) {
  std::string actual;
  if (!Md5File(stagedFile, &actual))
    return "read_staged_failed";
  if (actual != expectedMd5)
    return "md5_mismatch";

  // 先清掉旧的待应用文件, 避免把上一轮的包误当成这一轮
  unlink(PendingMd5().c_str());
  if (rename(stagedFile.c_str(), PendingBin().c_str()) != 0) {
    return std::string("rename_failed:") + strerror(errno);
  }
  chmod(PendingBin().c_str(), 0755);

  FILE *fp = fopen(PendingMd5().c_str(), "wb");
  if (fp == nullptr)
    return "marker_failed";
  fwrite(expectedMd5.data(), 1, expectedMd5.size(), fp);
  fflush(fp);
  fclose(fp);

  MAICRO_INFO("upgrade: pending ready (md5=%s, %s)",
              expectedMd5.c_str(), stagedFile.c_str());
  return std::string();
}

void DiscardPending() {
  unlink(PendingBin().c_str());
  unlink(PendingMd5().c_str());
}

void ApplyPendingAndExec(int argc, char *argv[]) {
  // exec 后新进程会重新从 main 进入; 用环境变量保证只应用一次, 不会无限 exec
  if (getenv("M3_UPGRADE_APPLIED") != nullptr)
    return;

  std::string expected;
  if (!ReadMd5Marker(&expected))
    return; // 没有待升级标记: 正常启动

  std::string actual;
  if (!Md5File(PendingBin(), &actual)) {
    MAICRO_ERROR("upgrade: pending.bin 读取失败, 放弃本次升级");
    return;
  }
  if (actual != expected) {
    // 不删除: 留给人工排查; 下次仍会校验失败, 不会误替换
    MAICRO_ERROR("upgrade: pending md5 不符(期望 %s, 实际 %s), 放弃本次升级",
                 expected.c_str(), actual.c_str());
    return;
  }

  std::string self = SelfExePath();
  if (self.empty()) {
    MAICRO_ERROR("upgrade: 取不到当前程序路径, 放弃本次升级");
    return;
  }

  std::string backup = self + ".bak." + std::to_string((long long)time(nullptr));
  if (!CopyFile(self, backup)) {
    MAICRO_ERROR("upgrade: 备份当前程序失败(%s), 放弃本次升级", backup.c_str());
    return;
  }

  if (rename(PendingBin().c_str(), self.c_str()) != 0) {
    MAICRO_ERROR("upgrade: 替换程序失败 errno=%d(%s)", errno, strerror(errno));
    return;
  }
  chmod(self.c_str(), 0755);
  unlink(PendingMd5().c_str());

  // 只保留最近 3 份备份(用 shell 排序, 避免引入目录遍历代码)
  std::string prune = "ls -1t '" + self + "'.bak.* 2>/dev/null | tail -n +4 | xargs -r rm -f";
  if (system(prune.c_str()) != 0) {
    // 清理失败不影响升级本身
  }

  MAICRO_INFO("upgrade: 已应用新程序(%s), 重新执行", self.c_str());
  fflush(stdout);
  setenv("M3_UPGRADE_APPLIED", "1", 1);
  if (argc > 0 && argv != nullptr) {
    execv(self.c_str(), argv);
    MAICRO_ERROR("upgrade: execv 失败 errno=%d(%s), 继续运行旧程序", errno,
                 strerror(errno));
  }
}

} // namespace upgrade
} // namespace maicro
