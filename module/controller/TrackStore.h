#pragma once
#include <stdint.h>

#include <mutex>
#include <string>
#include <vector>

namespace maicro {

/**
 * 单个航点: 采集时刻的无人机位姿 + 云台姿态 + 定位质量。
 *
 * 高度存两份且都必须保存, 回放时二者互相校验:
 *   altEll —— 椭球高(m), 来自 POSITION_FUSED, 是 FlyToPosition 接口要求的量纲;
 *   relH   —— 相对起飞点高(m), 来自 QueryDroneRelativeHeight, 语义直观、便于配置限幅。
 * 回放时用 "当前椭球高 + (目标relH - 当前relH)" 换算目标椭球高(与 DoFlyToTarget 一致),
 * 再与 altEll 交叉比对; 偏差超限说明换了起飞点或 RTK 基准漂移, 必须拒飞而不是将错就错。
 */
struct S_TrackPoint {
  uint16_t idx;       // 1-based 航点序号
  double lat;         // 纬度, 度
  double lon;         // 经度, 度
  double altEll;      // 椭球高, m
  float relH;         // 相对起飞点高, m
  float gPitch;       // 云台俯仰, 度
  float gRoll;        // 云台横滚, 度
  float gYaw;         // 云台偏航, 度
  float dYaw;         // 机头偏航, 度
  uint16_t satNum;    // 采集时可见卫星数
  uint8_t rtkSol;     // 采集时 RTK 解状态(50 = NARROW_INT 固定解)
  int64_t tsMs;       // 采集时刻, epoch 毫秒
};

/** 一条航迹: 名称 + 创建时间 + 有序航点 */
struct S_Track {
  std::string name;
  int64_t createdAt;  // epoch 秒
  std::vector<S_TrackPoint> pts;
};

/** 航迹列表条目(供 LIST 分页回传, 不加载航点以免读大文件) */
struct S_TrackItem {
  std::string name;
  int pointCount;
  int64_t createdAt;
};

/**
 * 航迹存储: 采集会话缓冲 + 文件读写 + 完整性校验。
 *
 * ==== 设计要点 ====
 *
 * 1. 文件格式为手写行式文本, 不用 JSON(工程 3rd_party 只有 spdlog, 无 JSON 库),
 *    也不复用 TCfgFileParser: 后者 section 用 std::map 按字典序输出, 会把 WP10 排到
 *    WP2 之前; 且其 extractKeyValue 用 strrchr(';') 截断, 值内含 ';' 会损坏数据。
 *
 * 2. 带 FNV-1a 32 位校验和: 文件被改坏或掉电写半截时整文件拒绝(ERROR:corrupt),
 *    绝不带着可疑坐标去飞 —— 坐标错一位小数就是几十公里外的目标点。
 *
 * 3. 写入用 .tmp + fsync + rename: rename 在同一文件系统内是原子的,
 *    掉电只会留下完整的旧文件或完整的新文件, 不会出现半截。
 *
 * 4. 采集会话只存内存(不落盘): 未 SAVE 的航点在服务进程重启后丢失, 这是有意为之 ——
 *    避免把"采到一半的航迹"当成正式航迹误执行。
 *
 * 5. 所有方法线程安全。调用约定: 文件 IO 只应在 MOP 线程调用, 绝不进飞行循环,
 *    否则会阻塞 TaskWorker 导致降落/返航无法及时执行。
 */
class TrackStore {
 public:
  /** 名称长度上限(字节)。限制它是为了保证 MOP 单行响应不超长 */
  static const size_t kNameMaxBytes = 24;

  /** 单条航迹最大航点数, 超出则拒绝采集/保存 */
  static const size_t kMaxPoints = 200;

  /** RTK 窄区固定解(NARROW_INT), 唯一被认可为"RTK 可用"的取值 */
  static const uint8_t kRtkFixed = 50;

  TrackStore();
  ~TrackStore();

  /**
   * 初始化: 创建存储目录。
   * @return 0 成功; -1 目录不可创建或不可写。
   *         失败时 dirReady_ 为 false, 后续 SaveTrack 一律返回 -4(io),
   *         **不会静默丢弃航迹**。
   */
  int Init(const std::string &dir);

  bool IsReady() const { return dirReady_; }

  // ==================== 采集会话 ====================

  /** 清空当前采集会话, 返回清空后的点数(恒为 0) */
  int BeginSession();

  /**
   * 把当前位姿作为一个航点加入会话缓冲。
   * @return >=1 新航点的 1-based 序号;
   *         -1 会话已满(kMaxPoints);
   *         -2 位置无效(经纬度全零或超范围);
   *         -3 RTK 非固定解(采到的点本身不准, 无回放价值, 拒绝记录);
   *         -4 相对高度超出 [1,500] 合理区间。
   */
  int AddPoint(const S_TrackPoint &pt);

  /** 当前会话已采集的航点数 */
  int SessionPointCount();

  // ==================== 文件操作 ====================

  /**
   * 把会话保存为一条命名航迹。
   * @param force 同名航迹是否允许覆盖
   * @return 0 成功; -1 名称非法; -2 会话为空; -3 同名已存在(force=false);
   *         -4 目录不可写等 IO 错误; -5 序列化内容自校验失败。
   */
  int SaveTrack(const std::string &name, bool force);

  /**
   * 加载并校验一条航迹。
   * @return 0 成功; -1 名称非法; -2 文件不存在; -3 文件损坏(校验和或字段不合法)。
   */
  int LoadTrack(const std::string &name, S_Track &out);

  /**
   * 枚举已有航迹(按名称字典序, 不加载航点)。
   * @param out 全部条目
   * @return 0 成功; -4 目录不可读。
   */
  int ListTracks(std::vector<S_TrackItem> &out);

  /**
   * 删除一条航迹。
   * @return 0 成功; -1 名称非法; -2 文件不存在; -4 删除失败。
   */
  int DeleteTrack(const std::string &name);

  /** 航迹是否存在 */
  bool Exists(const std::string &name);

  // ==================== 工具(供 MopCommandServer 复用) ====================

  /**
   * 名称合法性校验: 1~kNameMaxBytes 字节, 且不含会破坏行协议或文件解析的字符。
   * 禁用: '\r' '\n'(破坏单行协议)、','(字段分隔符)、'|'(备用分隔符)、
   *       '%'(转义引导符)、空格(文件头是空格分隔字段)、'/' 与 '.'(路径穿越与扩展名伪造)。
   * 允许中文: UTF-8 续字节 >= 0x80, 不与上述 ASCII 分隔符冲突。
   */
  static bool IsValidName(const std::string &name);

  /**
   * 百分号转义: 仅对协议敏感字符编码(%2C %25 %0A %0D %7C %20 %2F %2E)。
   * 中文等多字节字符原样保留, 既省字节又免去双向转码。
   * 线上协议与文件头共用同一套编解码, 避免两处规则不一致。
   */
  static std::string EncodeName(const std::string &raw);
  static std::string DecodeName(const std::string &enc);

  /**
   * 航迹中是否存在非固定解采集的航点。
   * 回放前调用: 历史点若不是固定解采的, 坐标本身就不可信, 应拒绝执行整条航迹。
   */
  static bool HasNonFixedRtkPoint(const S_Track &t);

 private:
  std::string FilePath(const std::string &name) const;

  /** 序列化航迹为文件内容(不含末尾换行以外的修饰) */
  static std::string Serialize(const S_Track &t);

  /** 解析并校验文件内容; 失败返回 -3 并填充 reason */
  static int Parse(const std::string &content, S_Track &out, std::string &reason);

  /** FNV-1a 32 位, 只对 WP 行计算, 头部变化(如重命名)不影响点数据校验 */
  static uint32_t Fnv1a(const std::string &s);

  /** 校验单个航点数值合理性 */
  static bool IsPointSane(const S_TrackPoint &p);

 private:
  std::string dir_;
  bool dirReady_;

  std::mutex sessionMutex_;              // 保护 session_
  std::vector<S_TrackPoint> session_;    // 采集会话缓冲(仅内存)

  std::mutex fileMutex_;                 // 保护目录读写与文件增删
};

}  // namespace maicro
