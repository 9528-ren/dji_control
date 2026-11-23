#include "ConfBase.h"

#include "MaicroLogger.inl"

namespace maicro {
ConfBase::ConfBase() {}

ConfBase::~ConfBase() {}

/**
 * 读取字符串配置项。缺失或为空时打 ERROR 并返回 init_val。
 * 缺失不致命: 上层各 Get*Para 调用都给了默认值, 段缺失只记日志继续启动。
 */
std::string ConfBase::GetStringPara(const std::string &s_type,
                                    const std::string &key_words,
                                    const std::string &init_val) {
  std::string rtn_s = "";
  rtn_s = cfg_.getValue(s_type, key_words);
  if (rtn_s == "") {
    MAICRO_ERROR("Parse config file error: [%s, %s]", s_type.c_str(),
                 key_words.c_str());
    rtn_s = init_val;
  }

  return rtn_s;
}

/**
 * 读取 float 配置项, 缺失时返回 init_val。
 *
 * 注意: init_val 形参类型是 int(既有签名), 因此**默认值的小数部分会被截断**。
 * 需要小数默认值或高精度(如经纬度)的项, 请用 ConfM3Gimbal::GetDoublePara,
 * 不要在此处传 2.0f 之类 —— 它会被隐式转成 2。
 */
float ConfBase::GetFloatPara(const std::string &s_type,
                             const std::string &key_words, const int init_val) {
  float rtn = 0;
  std::string value = "";
  value = cfg_.getValue(s_type, key_words);
  if (value == "") {
    MAICRO_ERROR("Parse config file error: [%s, %s]", s_type.c_str(),
                 key_words.c_str());
    rtn = init_val;
  } else {
    rtn = atof(value.c_str());
  }
  return rtn;
}

/// 读取 int 配置项, 缺失时打 ERROR 并返回 init_val。用 atoi, 非法字符串得 0。
int ConfBase::GetIntPara(const std::string &s_type,
                         const std::string &key_words, const int init_val) {
  int rtn = 0;
  std::string value = "";
  value = cfg_.getValue(s_type, key_words);
  if (value == "") {
    MAICRO_ERROR("Parse config file error: [%s, %s]", s_type.c_str(),
                 key_words.c_str());
    rtn = init_val;
  } else {
    rtn = atoi(value.c_str());
  }
  return rtn;
}

/**
 * 读取 bool 配置项。**只认字面 "true" / "false"**,
 * 其余取值(含 1/0/yes/no/空)一律打 ERROR 并返回 init_val。
 * 配置文件里写 1 表示真是不生效的, 必须写 true。
 */
bool ConfBase::GetBoolPara(const std::string &s_type,
                           const std::string &key_words, const bool init_val) {
  bool rtn = false;
  std::string value = "";
  value = cfg_.getValue(s_type, key_words);
  if (value == "false") {
    rtn = false;
  } else if (value == "true") {
    rtn = true;
  } else {
    MAICRO_ERROR("Parse config file error: [%s, %s]", s_type.c_str(),
                 key_words.c_str());
    rtn = init_val;
  }
  return rtn;
}
}  // namespace maicro