#include "ConfBase.h"

#include "MaicroLogger.inl"

namespace maicro {
ConfBase::ConfBase() {}

ConfBase::~ConfBase() {}

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