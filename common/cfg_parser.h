/*
 * @Description: Copyright (C), 2021-, airia Tech. Co., Ltd.
 * @version: V1.0
 * @Author: airia
 * @Create Date: Do not edit
 * @LastEditors: maicro
 * @LastEditTime: 2023-06-07 08:45:25
 */
#ifndef _CONFIG_H
#define _CONFIG_H

#if _MSC_VER > 1000
#pragma once
#endif  //   _MSC_VER   >   1000

#include <stdio.h>
#include <stdlib.h>

#ifdef WIN32
#pragma warning(disable : 4786)
#pragma warning(disable : 4503)
#endif

#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <string>
#include <vector>

namespace maicro {
class TCfgFileParser {
 public:
  TCfgFileParser(char chSectionBMark = '[', char chSectionEMark = ']',
                 char chRecordEMark = ';',  // record   end-mark
                 char chCommentMark = '#')
      : m_chSectionBMark(chSectionBMark),
        m_chSectionEMark(chSectionEMark),
        m_chRecordEMark(chRecordEMark),
        m_chCommentMark(chCommentMark) {}

  virtual ~TCfgFileParser() {}

  typedef std::map<std::string, std::string> SECTION_CONTENT;
  typedef std::map<std::string, SECTION_CONTENT> FILE_CONTENT;

 private:
  enum {
    SYNTAX_ERROR = 0x100,
  };

 public:
  const char* getErrorString(int err);
  int parseFile(const char* lpszFilename);
  void SaveFile(const char* lpszFilename);
  int sections() const { return m_content.size(); }

  FILE_CONTENT::const_iterator sectionBegin() const {
    return m_content.begin();
  }
  FILE_CONTENT::const_iterator sectionEnd() const { return m_content.end(); }

  // return   empty-string   if   no   corresponding   value   presented.

  std::string getValue(const std::string& strSection,
                       const std::string& strKey) const;
  // std::string   getValue(const std::string& strSection, const int num,const
  // std::string& strKey) const;
  void printContent();

  int getKeyNum();
  FILE_CONTENT m_content;

 protected:
  bool extractSection(char* line, std::string& strSection);
  bool extractKeyValue(char* line,
                       std::pair<std::string, std::string>& pairKeyValue);

 private:
  char m_chSectionBMark;
  char m_chSectionEMark;
  char m_chRecordEMark;
  char m_chCommentMark;
  int m_error_line;
};
}  // namespace maicro
#endif
