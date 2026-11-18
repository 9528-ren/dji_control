// TrackStore 独立单元验证(主机 x86 编译, 不依赖 PSDK)
// 覆盖: 会话采集门控、保存/加载往返一致、校验和防篡改、名称校验与转义、
//       列表分页、删除、损坏文件拒绝、同名覆盖策略
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include <string>
#include <vector>

#include "TrackStore.h"

using namespace maicro;

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg)                                            \
  do {                                                              \
    if (cond) { g_pass++; printf("  PASS  %s\n", msg); }            \
    else { g_fail++; printf("  FAIL  %s   (line %d)\n", msg, __LINE__); } \
  } while (0)

static S_TrackPoint MakePt(uint16_t idx, double lat, double lon, float relH,
                           uint8_t rtk = TrackStore::kRtkFixed) {
  S_TrackPoint p;
  memset(&p, 0, sizeof(p));
  p.idx = idx;
  p.lat = lat;
  p.lon = lon;
  p.altEll = 45.0 + relH;
  p.relH = relH;
  p.gPitch = -30.0f;
  p.gRoll = 0.0f;
  p.gYaw = 12.5f;
  p.dYaw = 94.0f;
  p.satNum = 22;
  p.rtkSol = rtk;
  p.tsMs = 1763271234123LL;
  return p;
}

int main() {
  const char *dir = "/tmp/trk_test";
  ::system("rm -rf /tmp/trk_test");

  printf("\n=== 1. Init ===\n");
  TrackStore st;
  CHECK(st.Init(dir) == 0, "Init 成功");
  CHECK(st.IsReady(), "IsReady 为真");

  TrackStore bad;
  CHECK(bad.Init("/proc/nonexist/deep/path") != 0, "Init 非法目录返回失败");
  CHECK(!bad.IsReady(), "失败后 IsReady 为假");
  // 目录不可用时 SaveTrack 必须报 io, 不能静默丢数据
  bad.AddPoint(MakePt(1, 22.5, 113.9, 30.0f));
  CHECK(bad.SaveTrack("x", false) == -4, "目录未就绪时 SAVE 返回 io 错误");

  printf("\n=== 2. 名称校验 ===\n");
  CHECK(TrackStore::IsValidName("track1"), "字母数字合法");
  CHECK(TrackStore::IsValidName("巡检A线"), "中文合法");
  CHECK(!TrackStore::IsValidName(""), "空名非法");
  CHECK(!TrackStore::IsValidName("a b"), "含空格非法");
  CHECK(!TrackStore::IsValidName("a,b"), "含逗号非法");
  CHECK(!TrackStore::IsValidName("a%b"), "含百分号非法");
  CHECK(!TrackStore::IsValidName("a|b"), "含竖线非法");
  CHECK(!TrackStore::IsValidName("../etc/passwd"), "路径穿越非法");
  CHECK(!TrackStore::IsValidName("a.b"), "含点号非法");
  CHECK(!TrackStore::IsValidName(std::string(40, 'a')), "超长非法");
  CHECK(TrackStore::IsValidName(std::string(24, 'a')), "24字符合法");
  // 转义往返
  CHECK(TrackStore::DecodeName(TrackStore::EncodeName("a b,c")) == "a b,c",
        "转义往返一致(含空格逗号)");
  CHECK(TrackStore::DecodeName(TrackStore::EncodeName("巡检A")) == "巡检A",
        "中文转义往返一致");

  printf("\n=== 3. 采集门控 ===\n");
  st.BeginSession();
  CHECK(st.SessionPointCount() == 0, "BEGIN 后会话为空");
  CHECK(st.AddPoint(MakePt(1, 22.54281979, 113.95889718, 30.0f)) == 1, "正常点返回序号1");
  CHECK(st.AddPoint(MakePt(2, 22.54282100, 113.95889800, 35.0f)) == 2, "第二点返回序号2");
  CHECK(st.AddPoint(MakePt(3, 0.0000005, -0.00000009, 8.18f)) == -2, "未定位坐标被拒");
  CHECK(st.AddPoint(MakePt(4, 22.5, 113.9, 30.0f, 34)) == -3, "非固定解被拒");
  CHECK(st.AddPoint(MakePt(5, 22.5, 113.9, 0.5f)) == -4, "相对高过低被拒");
  CHECK(st.AddPoint(MakePt(6, 22.5, 113.9, 999.0f)) == -4, "相对高过高被拒");
  CHECK(st.SessionPointCount() == 2, "非法点未入会话, 仍为2");

  printf("\n=== 4. 保存 ===\n");
  CHECK(st.SaveTrack("", false) == -1, "空名保存被拒");
  CHECK(st.SaveTrack("t1", false) == 0, "保存 t1 成功");
  CHECK(st.Exists("t1"), "t1 存在");
  CHECK(st.SaveTrack("t1", false) == -3, "同名不覆盖返回 exists");
  CHECK(st.SaveTrack("t1", true) == 0, "FORCE 覆盖成功");

  TrackStore empty;
  empty.Init("/tmp/trk_test_empty");
  CHECK(empty.SaveTrack("e1", false) == -2, "空会话保存返回 empty");

  printf("\n=== 5. 加载往返一致 ===\n");
  S_Track t;
  CHECK(st.LoadTrack("t1", t) == 0, "加载 t1 成功");
  CHECK(t.name == "t1", "名称一致");
  CHECK(t.pts.size() == 2, "点数一致(2)");
  if (t.pts.size() == 2) {
    CHECK(t.pts[0].idx == 1 && t.pts[1].idx == 2, "序号连续");
    // 坐标 %.7f 精度约 1.1cm, 容差 1e-6 度
    CHECK(fabs(t.pts[0].lat - 22.54281979) < 1e-6, "纬度往返一致");
    CHECK(fabs(t.pts[0].lon - 113.95889718) < 1e-6, "经度往返一致");
    CHECK(fabs(t.pts[0].relH - 30.0f) < 0.01f, "相对高往返一致");
    CHECK(fabs(t.pts[0].gPitch - (-30.0f)) < 0.01f, "云台俯仰往返一致");
    CHECK(fabs(t.pts[0].gYaw - 12.5f) < 0.01f, "云台偏航往返一致");
    CHECK(fabs(t.pts[0].dYaw - 94.0f) < 0.01f, "机头偏航往返一致");
    CHECK(t.pts[0].satNum == 22, "卫星数往返一致");
    CHECK(t.pts[0].rtkSol == 50, "RTK解往返一致");
    CHECK(t.pts[0].tsMs == 1763271234123LL, "时间戳往返一致");
  }
  CHECK(!TrackStore::HasNonFixedRtkPoint(t), "全固定解航迹通过回放前校验");

  printf("\n=== 6. 中文名航迹往返 ===\n");
  st.BeginSession();
  st.AddPoint(MakePt(1, 22.5, 113.9, 30.0f));
  CHECK(st.SaveTrack("巡检A线", false) == 0, "中文名保存成功");
  S_Track tc;
  CHECK(st.LoadTrack("巡检A线", tc) == 0, "中文名加载成功");
  CHECK(tc.name == "巡检A线", "中文名往返一致");

  printf("\n=== 7. 防篡改: 校验和 ===\n");
  {
    std::string path = std::string(dir) + "/t1.trk";
    // 篡改坐标一位小数(把 22.5428198 改成 23.5428198)
    std::string cmd = "sed -i 's/22.5428198/23.5428198/' " + path;
    ::system(cmd.c_str());
    S_Track tt;
    CHECK(st.LoadTrack("t1", tt) == -3, "篡改坐标后判为 corrupt 并拒绝加载");
  }
  {
    // 恢复: 重新保存一份干净的
    st.BeginSession();
    st.AddPoint(MakePt(1, 22.54281979, 113.95889718, 30.0f));
    st.AddPoint(MakePt(2, 22.54282100, 113.95889800, 35.0f));
    CHECK(st.SaveTrack("t1", true) == 0, "重新保存干净副本");
    S_Track tt;
    CHECK(st.LoadTrack("t1", tt) == 0 && tt.pts.size() == 2, "干净副本可正常加载");
  }
  {
    // 截断文件(模拟掉电写半截)
    std::string path = std::string(dir) + "/t1.trk";
    ::truncate(path.c_str(), 60);
    S_Track tt;
    CHECK(st.LoadTrack("t1", tt) == -3, "截断文件判为 corrupt");
  }
  CHECK(st.LoadTrack("nonexist", t) == -2, "不存在返回 notfound");
  CHECK(st.LoadTrack("../etc/passwd", t) == -1, "非法名返回 badname");

  printf("\n=== 8. 列表与分页数据 ===\n");
  {
    // 重建 t1
    st.BeginSession();
    st.AddPoint(MakePt(1, 22.54281979, 113.95889718, 30.0f));
    st.AddPoint(MakePt(2, 22.54282100, 113.95889800, 35.0f));
    st.SaveTrack("t1", true);
  }
  std::vector<S_TrackItem> items;
  CHECK(st.ListTracks(items) == 0, "ListTracks 成功");
  CHECK(items.size() == 2, "列表含 2 条(t1 + 巡检A线)");
  bool sorted = true;
  for (size_t i = 1; i < items.size(); ++i)
    if (!(items[i - 1].name < items[i].name)) sorted = false;
  CHECK(sorted, "列表按名称排序(分页结果稳定)");
  bool foundT1 = false;
  for (size_t i = 0; i < items.size(); ++i) {
    if (items[i].name == "t1") {
      foundT1 = true;
      CHECK(items[i].pointCount == 2, "t1 点数正确(2)");
      CHECK(items[i].createdAt > 1700000000LL, "t1 创建时间合理");
    }
  }
  CHECK(foundT1, "列表含 t1");

  printf("\n=== 9. 损坏文件仍应出现在列表(便于用户删除) ===\n");
  // 设计约定: LIST 只读文件首行(不加载航点、不验校验和), 因此它报出的是头部声明的
  // 点数, 属**未经校验的展示值**; 权威校验在 LoadTrack(GET/EXEC 都走它), 损坏必被拒。
  // 这样用户打开列表不必付出"读并校验全部文件"的代价, 而任何实际使用仍受保护。
  {
    std::string p = std::string(dir) + "/broken.trk";
    FILE *fp = fopen(p.c_str(), "w");
    fprintf(fp, "M3TRACK v=1 n=99 ts=0 sum=deadbeef name=broken\nGARBAGE LINE\n");
    fclose(fp);
  }
  items.clear();
  CHECK(st.ListTracks(items) == 0, "含损坏文件时 ListTracks 仍成功");
  bool brokenListed = false;
  for (size_t i = 0; i < items.size(); ++i)
    if (items[i].name == "broken") {
      brokenListed = true;
      CHECK(items[i].pointCount == 99, "损坏项 pointCount 取头部声明值(展示用, 未校验)");
      CHECK(items[i].createdAt == 0, "损坏项 createdAt 取头部值");
    }
  CHECK(brokenListed, "损坏文件出现在列表中(用户可删除)");
  S_Track tb;
  CHECK(st.LoadTrack("broken", tb) == -3, "损坏文件加载被拒(权威校验在 LOAD)");

  // 头部不可解析时(连首行都坏), LIST 才标记为 -1
  {
    std::string p = std::string(dir) + "/headless.trk";
    FILE *fp = fopen(p.c_str(), "w");
    fprintf(fp, "THIS IS NOT A TRACK FILE\n");
    fclose(fp);
  }
  items.clear();
  st.ListTracks(items);
  bool headlessListed = false;
  for (size_t i = 0; i < items.size(); ++i)
    if (items[i].name == "headless") { headlessListed = true; CHECK(items[i].pointCount == -1, "首行不可解析时标记 -1"); }
  CHECK(headlessListed, "首行损坏的文件仍列出(可删除)");
  CHECK(st.DeleteTrack("headless") == 0, "删除首行损坏文件成功");

  printf("\n=== 10. 删除 ===\n");
  CHECK(st.DeleteTrack("broken") == 0, "删除损坏文件成功");
  CHECK(!st.Exists("broken"), "删除后不存在");
  CHECK(st.DeleteTrack("broken") == -2, "重复删除返回 notfound");
  CHECK(st.DeleteTrack("../x") == -1, "非法名删除返回 badname");
  items.clear();
  st.ListTracks(items);
  CHECK(items.size() == 2, "删除后列表剩 2 条");

  printf("\n=== 11. 点数上限 ===\n");
  st.BeginSession();
  int added = 0;
  for (int i = 0; i < 250; ++i) {
    if (st.AddPoint(MakePt((uint16_t)(i + 1), 22.5 + i * 1e-6, 113.9 + i * 1e-6, 30.0f)) > 0) added++;
  }
  CHECK(added == (int)TrackStore::kMaxPoints, "超过上限的点被拒绝(只收 200)");

  printf("\n=== 12. BEGIN 清空未保存会话 ===\n");
  CHECK(st.SessionPointCount() == 200, "清空前 200 点");
  st.BeginSession();
  CHECK(st.SessionPointCount() == 0, "BEGIN 后清空");

  printf("\n========================================\n");
  printf("  通过 %d 项, 失败 %d 项\n", g_pass, g_fail);
  printf("========================================\n");
  ::system("rm -rf /tmp/trk_test /tmp/trk_test_empty");
  return g_fail == 0 ? 0 : 1;
}
