#include "TrackStore.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <sstream>

#include "MaicroLogger.inl"

namespace maicro {

// static const 成员的类外定义: MAICRO_* 日志宏经 log_printf(..., const Args&...)
// 会绑定引用, 构成 ODR-use, 缺定义会链接失败(undefined reference)。
const size_t TrackStore::kNameMaxBytes;
const size_t TrackStore::kMaxPoints;
const uint8_t TrackStore::kRtkFixed;

namespace {

const char *kFileExt = ".trk";
const int kHeaderVersion = 1;

/** 每个 WP 行的字段数: WP + idx + lat + lon + altEll + relH + gPitch + gRoll + gYaw + dYaw + sat + rtkSol + tsMs */
const size_t kWpFieldCount = 13;

/** 相对高度合理区间(m)。低于 1m 会贴地撞障碍, 高于 500m 远超本机型作业范围 */
const float kRelHMin = 1.0f;
const float kRelHMax = 500.0f;

/** 按空格切分一行, 返回 token 列表 */
std::vector<std::string> SplitSpace(const std::string &line) {
  std::vector<std::string> out;
  std::istringstream iss(line);
  std::string tok;
  while (iss >> tok) out.push_back(tok);
  return out;
}

/** 严格解析浮点: 必须整串消费完, 否则视为损坏(防止 "12.3abc" 被静默截断) */
bool ParseDouble(const std::string &s, double &out) {
  if (s.empty()) return false;
  errno = 0;
  char *end = nullptr;
  double v = strtod(s.c_str(), &end);
  if (end != s.c_str() + s.size() || *end != '\0') return false;
  if (errno == ERANGE) return false;
  out = v;
  return true;
}

/** 严格解析无符号整数 */
bool ParseUint(const std::string &s, unsigned long &out) {
  if (s.empty()) return false;
  errno = 0;
  char *end = nullptr;
  unsigned long v = strtoul(s.c_str(), &end, 10);
  if (end != s.c_str() + s.size() || *end != '\0') return false;
  if (errno == ERANGE) return false;
  out = v;
  return true;
}

/** 严格解析有符号 64 位整数(时间戳) */
bool ParseInt64(const std::string &s, long long &out) {
  if (s.empty()) return false;
  errno = 0;
  char *end = nullptr;
  long long v = strtoll(s.c_str(), &end, 10);
  if (end != s.c_str() + s.size() || *end != '\0') return false;
  if (errno == ERANGE) return false;
  out = v;
  return true;
}

/** 从头部的 "key=value" 里取值 */
bool HeaderField(const std::vector<std::string> &toks, const char *key,
                 std::string &out) {
  size_t klen = strlen(key);
  for (size_t i = 1; i < toks.size(); ++i) {
    if (toks[i].size() > klen && toks[i].compare(0, klen, key) == 0 &&
        toks[i][klen] == '=') {
      out = toks[i].substr(klen + 1);
      return true;
    }
  }
  return false;
}

/** 只读文件第一行(用于 LIST 时取点数与创建时间, 避免加载整条航迹) */
bool ReadFirstLine(const std::string &path, std::string &line) {
  FILE *fp = fopen(path.c_str(), "r");
  if (!fp) return false;
  char buf[1024];
  bool ok = false;
  if (fgets(buf, sizeof(buf), fp) != nullptr) {
    line = buf;
    while (!line.empty() && (line[line.size() - 1] == '\n' ||
                              line[line.size() - 1] == '\r')) {
      line.erase(line.size() - 1);
    }
    ok = true;
  }
  fclose(fp);
  return ok;
}

}  // namespace

TrackStore::TrackStore() : dirReady_(false) {}

TrackStore::~TrackStore() {}

int TrackStore::Init(const std::string &dir) {
  dir_ = dir;
  dirReady_ = false;
  if (dir_.empty()) {
    MAICRO_ERROR("track dir is empty");
    return -1;
  }

  if (::mkdir(dir_.c_str(), 0755) != 0 && errno != EEXIST) {
    MAICRO_ERROR("create track dir '%s' failed: %s", dir_.c_str(),
                 strerror(errno));
    return -1;
  }

  struct stat st;
  if (::stat(dir_.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
    MAICRO_ERROR("track path '%s' is not a directory", dir_.c_str());
    return -1;
  }
  if (::access(dir_.c_str(), W_OK | X_OK) != 0) {
    MAICRO_ERROR("track dir '%s' not writable: %s", dir_.c_str(),
                 strerror(errno));
    return -1;
  }

  dirReady_ = true;
  MAICRO_INFO("track store ready, dir=%s maxPoints=%zu", dir_.c_str(),
              kMaxPoints);
  return 0;
}

// ==================== 采集会话 ====================

int TrackStore::BeginSession() {
  std::lock_guard<std::mutex> lk(sessionMutex_);
  size_t old = session_.size();
  session_.clear();
  if (old > 0) {
    MAICRO_WARN("track session cleared, %zu unsaved points discarded", old);
  }
  return 0;
}

int TrackStore::AddPoint(const S_TrackPoint &pt) {
  std::lock_guard<std::mutex> lk(sessionMutex_);

  if (session_.size() >= kMaxPoints) {
    MAICRO_ERROR("track session full (%zu), reject new point", kMaxPoints);
    return -1;
  }
  // 位置无效: 经纬度全零说明飞控还没定位(实测过 lat 0.00000050 这种未定位值),
  // 存下来回放时会让飞机飞向赤道原点, 必须拒绝
  if (!(fabs(pt.lat) <= 90.0 && fabs(pt.lon) <= 180.0) ||
      (fabs(pt.lat) < 1e-6 && fabs(pt.lon) < 1e-6)) {
    MAICRO_ERROR("invalid position lat %.8f lon %.8f, reject point", pt.lat,
                 pt.lon);
    return -2;
  }
  // RTK 非固定解时采的点精度只到米级甚至更差, 回放没有价值, 直接拒绝
  if (pt.rtkSol != kRtkFixed) {
    MAICRO_WARN("rtk solution %u != %u(NARROW_INT), reject point", pt.rtkSol,
                kRtkFixed);
    return -3;
  }
  if (!(pt.relH >= kRelHMin && pt.relH <= kRelHMax)) {
    MAICRO_ERROR("relative height %.2f out of [%.0f,%.0f], reject point",
                 pt.relH, kRelHMin, kRelHMax);
    return -4;
  }

  S_TrackPoint p = pt;
  p.idx = (uint16_t)(session_.size() + 1);
  session_.push_back(p);
  MAICRO_INFO("track point #%u recorded: lat %.7f lon %.7f altEll %.3f "
              "relH %.2f gPitch %.2f sat %u rtk %u",
              p.idx, p.lat, p.lon, p.altEll, p.relH, p.gPitch, p.satNum,
              p.rtkSol);
  return (int)p.idx;
}

int TrackStore::SessionPointCount() {
  std::lock_guard<std::mutex> lk(sessionMutex_);
  return (int)session_.size();
}

// ==================== 文件操作 ====================

std::string TrackStore::FilePath(const std::string &name) const {
  return dir_ + "/" + name + kFileExt;
}

bool TrackStore::IsValidName(const std::string &name) {
  if (name.empty() || name.size() > kNameMaxBytes) return false;
  for (size_t i = 0; i < name.size(); ++i) {
    unsigned char c = (unsigned char)name[i];
    // 控制字符与 DEL
    if (c < 0x20 || c == 0x7F) return false;
    switch (c) {
      case '\r':
      case '\n':  // 破坏单行协议
      case ',':   // 字段分隔符
      case '|':   // 备用分隔符
      case '%':   // 转义引导符
      case ' ':   // 文件头是空格分隔字段
      case '/':   // 路径穿越
      case '\\':
      case '.':   // 路径穿越与扩展名伪造
        return false;
      default:
        break;
    }
  }
  return true;
}

std::string TrackStore::EncodeName(const std::string &raw) {
  std::string out;
  out.reserve(raw.size() + 8);
  for (size_t i = 0; i < raw.size(); ++i) {
    char c = raw[i];
    switch (c) {
      case '%': out += "%25"; break;
      case ',': out += "%2C"; break;
      case '\n': out += "%0A"; break;
      case '\r': out += "%0D"; break;
      case '|': out += "%7C"; break;
      case ' ': out += "%20"; break;
      case '/': out += "%2F"; break;
      case '.': out += "%2E"; break;
      default: out += c; break;
    }
  }
  return out;
}

std::string TrackStore::DecodeName(const std::string &enc) {
  std::string out;
  out.reserve(enc.size());
  for (size_t i = 0; i < enc.size(); ++i) {
    if (enc[i] == '%' && i + 2 < enc.size()) {
      std::string hex = enc.substr(i + 1, 2);
      char *end = nullptr;
      unsigned long v = strtoul(hex.c_str(), &end, 16);
      if (end == hex.c_str() + 2) {
        out += (char)(unsigned char)v;
        i += 2;
        continue;
      }
    }
    out += enc[i];
  }
  return out;
}

bool TrackStore::IsPointSane(const S_TrackPoint &p) {
  if (!(fabs(p.lat) <= 90.0 && fabs(p.lon) <= 180.0)) return false;
  if (fabs(p.lat) < 1e-6 && fabs(p.lon) < 1e-6) return false;  // 未定位
  if (!(p.relH >= kRelHMin && p.relH <= kRelHMax)) return false;
  if (p.gPitch < -180.0f || p.gPitch > 180.0f) return false;
  return true;
}

uint32_t TrackStore::Fnv1a(const std::string &s) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < s.size(); ++i) {
    h ^= (uint8_t)s[i];
    h *= 16777619u;
  }
  return h;
}

std::string TrackStore::Serialize(const S_Track &t) {
  // 先拼 WP 块, 校验和只覆盖它 —— 重命名航迹不需要改点数据校验和
  std::string wpBlock;
  char line[512];
  for (size_t i = 0; i < t.pts.size(); ++i) {
    const S_TrackPoint &p = t.pts[i];
    int n = snprintf(line, sizeof(line),
                     "WP %u %.7f %.7f %.3f %.3f %.2f %.2f %.2f %.2f %u %u "
                     "%lld\n",
                     (unsigned)p.idx, p.lat, p.lon, p.altEll, (double)p.relH,
                     (double)p.gPitch, (double)p.gRoll, (double)p.gYaw,
                     (double)p.dYaw, (unsigned)p.satNum, (unsigned)p.rtkSol,
                     (long long)p.tsMs);
    if (n <= 0 || (size_t)n >= sizeof(line)) {
      MAICRO_ERROR("serialize WP line overflow at idx %u", (unsigned)p.idx);
      return std::string();
    }
    wpBlock += line;
  }

  char header[512];
  int hn = snprintf(header, sizeof(header),
                    "M3TRACK v=%d n=%zu ts=%lld sum=%08x name=%s\n",
                    kHeaderVersion, t.pts.size(), (long long)t.createdAt,
                    Fnv1a(wpBlock), EncodeName(t.name).c_str());
  if (hn <= 0 || (size_t)hn >= sizeof(header)) {
    MAICRO_ERROR("serialize header overflow, name='%s'", t.name.c_str());
    return std::string();
  }
  return std::string(header) + wpBlock;
}

int TrackStore::Parse(const std::string &content, S_Track &out,
                      std::string &reason) {
  // 按 '\n' 切行
  std::vector<std::string> lines;
  size_t start = 0;
  while (start <= content.size()) {
    size_t pos = content.find('\n', start);
    if (pos == std::string::npos) {
      if (start < content.size()) lines.push_back(content.substr(start));
      break;
    }
    lines.push_back(content.substr(start, pos - start));
    start = pos + 1;
  }
  if (lines.empty()) {
    reason = "empty file";
    return -3;
  }
  // 去掉行尾 '\r'(兼容被工具改过的文件)
  for (size_t i = 0; i < lines.size(); ++i) {
    while (!lines[i].empty() && lines[i][lines[i].size() - 1] == '\r') {
      lines[i].erase(lines[i].size() - 1);
    }
  }

  std::vector<std::string> h = SplitSpace(lines[0]);
  if (h.empty() || h[0] != "M3TRACK") {
    reason = "bad magic";
    return -3;
  }
  std::string sv, sn, sts, ssum, sname;
  if (!HeaderField(h, "v", sv) || !HeaderField(h, "n", sn) ||
      !HeaderField(h, "ts", sts) || !HeaderField(h, "sum", ssum)) {
    reason = "header field missing";
    return -3;
  }
  long long ver = 0, nPts = 0, ts = 0;
  if (!ParseInt64(sv, ver) || ver != kHeaderVersion) {
    reason = "unsupported version";
    return -3;
  }
  if (!ParseInt64(sn, nPts) || nPts < 0 || (size_t)nPts > kMaxPoints) {
    reason = "bad point count";
    return -3;
  }
  if (!ParseInt64(sts, ts)) {
    reason = "bad timestamp";
    return -3;
  }
  unsigned long sumFile = 0;
  // sum 是 8 位十六进制, **必须直接按 16 进制解析**。
  // 不可先用十进制试探: 校验和恰好全为数字时(如 12345678)十进制会解析"成功"
  // 但得到错误值, 导致合法航迹被误判为 corrupt 而拒绝起飞。
  if (ssum.size() != 8) {
    reason = "bad checksum field";
    return -3;
  }
  {
    char *end = nullptr;
    errno = 0;
    unsigned long v = strtoul(ssum.c_str(), &end, 16);
    if (end != ssum.c_str() + ssum.size() || errno == ERANGE) {
      reason = "bad checksum field";
      return -3;
    }
    sumFile = v;
  }
  // name= 放在头部最后, 允许值里含 '='(编码后不会出现, 但取最后一个更稳)
  if (!HeaderField(h, "name", sname)) {
    reason = "name missing";
    return -3;
  }

  // 重建 WP 块并校验校验和: 按文件里的原始行 + '\n' 拼接, 与序列化时逐字节一致
  std::string wpBlock;
  std::vector<S_TrackPoint> pts;
  for (size_t i = 1; i < lines.size(); ++i) {
    if (lines[i].empty()) continue;  // 忽略空行
    wpBlock += lines[i];
    wpBlock += "\n";

    std::vector<std::string> f = SplitSpace(lines[i]);
    if (f.size() != kWpFieldCount || f[0] != "WP") {
      reason = "wp field count";
      return -3;
    }
    S_TrackPoint p;
    unsigned long idx = 0, sat = 0, sol = 0;
    long long tsms = 0;
    double lat = 0, lon = 0, altEll = 0, relH = 0, gp = 0, gr = 0, gy = 0,
           dy = 0;
    if (!ParseUint(f[1], idx) || !ParseDouble(f[2], lat) ||
        !ParseDouble(f[3], lon) || !ParseDouble(f[4], altEll) ||
        !ParseDouble(f[5], relH) || !ParseDouble(f[6], gp) ||
        !ParseDouble(f[7], gr) || !ParseDouble(f[8], gy) ||
        !ParseDouble(f[9], dy) || !ParseUint(f[10], sat) ||
        !ParseUint(f[11], sol) || !ParseInt64(f[12], tsms)) {
      reason = "wp parse";
      return -3;
    }
    p.idx = (uint16_t)idx;
    p.lat = lat;
    p.lon = lon;
    p.altEll = altEll;
    p.relH = (float)relH;
    p.gPitch = (float)gp;
    p.gRoll = (float)gr;
    p.gYaw = (float)gy;
    p.dYaw = (float)dy;
    p.satNum = (uint16_t)sat;
    p.rtkSol = (uint8_t)sol;
    p.tsMs = (int64_t)tsms;
    pts.push_back(p);
  }

  if (Fnv1a(wpBlock) != (uint32_t)sumFile) {
    reason = "checksum mismatch";
    return -3;
  }
  if ((long long)pts.size() != nPts) {
    reason = "point count mismatch";
    return -3;
  }
  for (size_t i = 0; i < pts.size(); ++i) {
    // idx 必须从 1 连续递增, 乱序会导致回放顺序错误
    if (pts[i].idx != (uint16_t)(i + 1)) {
      reason = "idx not sequential";
      return -3;
    }
    if (!IsPointSane(pts[i])) {
      reason = "point out of range";
      return -3;
    }
  }

  out.name = DecodeName(sname);
  out.createdAt = (int64_t)ts;
  out.pts.swap(pts);
  return 0;
}

int TrackStore::SaveTrack(const std::string &name, bool force) {
  if (!IsValidName(name)) {
    MAICRO_ERROR("save track: invalid name '%s'", name.c_str());
    return -1;
  }

  S_Track t;
  {
    std::lock_guard<std::mutex> lk(sessionMutex_);
    if (session_.empty()) {
      MAICRO_WARN("save track: session empty");
      return -2;
    }
    t.name = name;
    t.pts = session_;
  }
  t.createdAt = (int64_t)time(nullptr);

  std::lock_guard<std::mutex> lk(fileMutex_);
  if (!dirReady_) {
    MAICRO_ERROR("save track: store not ready, refuse to lose data");
    return -4;
  }
  std::string path = FilePath(name);
  if (!force) {
    struct stat st;
    if (::stat(path.c_str(), &st) == 0) {
      MAICRO_WARN("save track '%s': already exists, use FORCE to overwrite",
                  name.c_str());
      return -3;
    }
  }

  std::string content = Serialize(t);
  if (content.empty()) {
    MAICRO_ERROR("save track '%s': serialize failed", name.c_str());
    return -5;
  }

  // 写临时文件 → fsync → rename: rename 在同一文件系统内原子,
  // 掉电只会留下完整旧文件或完整新文件
  std::string tmpPath = path + ".tmp";
  int fd = ::open(tmpPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    MAICRO_ERROR("save track '%s': open tmp failed: %s", name.c_str(),
                 strerror(errno));
    return -4;
  }
  size_t off = 0;
  while (off < content.size()) {
    ssize_t w = ::write(fd, content.data() + off, content.size() - off);
    if (w < 0) {
      if (errno == EINTR) continue;
      MAICRO_ERROR("save track '%s': write failed: %s", name.c_str(),
                   strerror(errno));
      ::close(fd);
      ::unlink(tmpPath.c_str());
      return -4;
    }
    off += (size_t)w;
  }
  if (::fsync(fd) != 0) {
    MAICRO_WARN("save track '%s': fsync failed: %s", name.c_str(),
                strerror(errno));
  }
  ::close(fd);

  if (::rename(tmpPath.c_str(), path.c_str()) != 0) {
    MAICRO_ERROR("save track '%s': rename failed: %s", name.c_str(),
                 strerror(errno));
    ::unlink(tmpPath.c_str());
    return -4;
  }
  // 尽力把目录项也落盘, 失败不致命(文件已就位)
  int dfd = ::open(dir_.c_str(), O_RDONLY);
  if (dfd >= 0) {
    ::fsync(dfd);
    ::close(dfd);
  }

  MAICRO_INFO("track saved: '%s' points=%zu path=%s force=%d", name.c_str(),
              t.pts.size(), path.c_str(), force ? 1 : 0);
  return 0;
}

int TrackStore::LoadTrack(const std::string &name, S_Track &out) {
  if (!IsValidName(name)) {
    MAICRO_ERROR("load track: invalid name '%s'", name.c_str());
    return -1;
  }
  std::lock_guard<std::mutex> lk(fileMutex_);
  std::string path = FilePath(name);

  FILE *fp = fopen(path.c_str(), "rb");
  if (!fp) {
    MAICRO_WARN("load track '%s': not found (%s)", name.c_str(), path.c_str());
    return -2;
  }
  std::string content;
  char buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) content.append(buf, n);
  fclose(fp);

  std::string reason;
  S_Track parsed;
  int ret = Parse(content, parsed, reason);
  if (ret != 0) {
    // 损坏文件绝不返回部分数据: 坐标可疑就不能拿去飞
    MAICRO_ERROR("load track '%s': CORRUPT (%s), refuse to use", name.c_str(),
                 reason.c_str());
    return -3;
  }
  out = parsed;
  MAICRO_INFO("track loaded: '%s' points=%zu createdAt=%lld", name.c_str(),
              out.pts.size(), (long long)out.createdAt);
  return 0;
}

int TrackStore::ListTracks(std::vector<S_TrackItem> &out) {
  out.clear();
  std::lock_guard<std::mutex> lk(fileMutex_);
  if (!dirReady_) {
    MAICRO_ERROR("list tracks: store not ready");
    return -4;
  }
  DIR *d = ::opendir(dir_.c_str());
  if (!d) {
    MAICRO_ERROR("list tracks: opendir '%s' failed: %s", dir_.c_str(),
                 strerror(errno));
    return -4;
  }
  size_t extLen = strlen(kFileExt);
  struct dirent *ent;
  while ((ent = ::readdir(d)) != nullptr) {
    std::string fn = ent->d_name;
    if (fn.size() <= extLen) continue;
    if (fn.compare(fn.size() - extLen, extLen, kFileExt) != 0) continue;
    std::string name = fn.substr(0, fn.size() - extLen);
    if (!IsValidName(name)) {
      // 目录里可能有手工放入的怪名字文件, 跳过而不是崩
      MAICRO_WARN("list tracks: skip invalid filename '%s'", fn.c_str());
      continue;
    }

    S_TrackItem item;
    item.name = name;
    item.pointCount = -1;  // -1 表示头部不可读/损坏, 仍列出以便用户删除
    item.createdAt = 0;

    std::string firstLine;
    if (ReadFirstLine(FilePath(name), firstLine)) {
      std::vector<std::string> h = SplitSpace(firstLine);
      std::string sn, sts;
      if (!h.empty() && h[0] == "M3TRACK" && HeaderField(h, "n", sn) &&
          HeaderField(h, "ts", sts)) {
        long long nPts = -1, ts = 0;
        if (ParseInt64(sn, nPts) && ParseInt64(sts, ts)) {
          item.pointCount = (int)nPts;
          item.createdAt = (int64_t)ts;
        }
      }
    }
    if (item.pointCount < 0) {
      MAICRO_WARN("list tracks: '%s' header unreadable/corrupt", name.c_str());
    }
    out.push_back(item);
  }
  closedir(d);

  // 按名称排序, 保证分页结果稳定(否则 readdir 顺序随文件系统变化)
  std::sort(out.begin(), out.end(),
            [](const S_TrackItem &a, const S_TrackItem &b) {
              return a.name < b.name;
            });
  MAICRO_INFO("list tracks: %zu entries in %s", out.size(), dir_.c_str());
  return 0;
}

int TrackStore::DeleteTrack(const std::string &name) {
  if (!IsValidName(name)) {
    MAICRO_ERROR("delete track: invalid name '%s'", name.c_str());
    return -1;
  }
  std::lock_guard<std::mutex> lk(fileMutex_);
  std::string path = FilePath(name);
  struct stat st;
  if (::stat(path.c_str(), &st) != 0) {
    MAICRO_WARN("delete track '%s': not found", name.c_str());
    return -2;
  }
  if (::unlink(path.c_str()) != 0) {
    MAICRO_ERROR("delete track '%s': unlink failed: %s", name.c_str(),
                 strerror(errno));
    return -4;
  }
  // 同名临时文件若存在(上次写入被打断)一并清掉, 避免残留
  std::string tmpPath = path + ".tmp";
  ::unlink(tmpPath.c_str());

  MAICRO_INFO("track deleted: '%s'", name.c_str());
  return 0;
}

bool TrackStore::Exists(const std::string &name) {
  if (!IsValidName(name)) return false;
  std::lock_guard<std::mutex> lk(fileMutex_);
  struct stat st;
  return ::stat(FilePath(name).c_str(), &st) == 0;
}

bool TrackStore::HasNonFixedRtkPoint(const S_Track &t) {
  for (size_t i = 0; i < t.pts.size(); ++i) {
    if (t.pts[i].rtkSol != kRtkFixed) return true;
  }
  return false;
}

}  // namespace maicro
