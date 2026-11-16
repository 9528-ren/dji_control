#pragma once
#include "cfg_parser.h"
#include <string>

namespace maicro {
class ConfBase {
 public:
  ConfBase();
  virtual ~ConfBase();
 
 virtual int ParseConfFile(const std::string &filePath) = 0;

  std::string GetStringPara(const std::string &s_type,
                            const std::string &key_words,
                            const std::string &init_val);

  float GetFloatPara(const std::string &s_type, const std::string &key_words,
                     const int init_val);

  int GetIntPara(const std::string &s_type, const std::string &key_words,
                 const int init_val);

  bool GetBoolPara(const std::string &s_type, const std::string &key_words,
                   const bool init_val);

 protected:
  TCfgFileParser cfg_;
};
}  // namespace maicro