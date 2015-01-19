#include "StdAfx_MOTracking.h"
#include "NHUtil.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

#ifdef _MSC_VER

#include <Windows.h>

std::string GetModulePath() {
  char sTemp[512];
  GetModuleFileName(NULL, sTemp, 512);
  for (int i=strlen(sTemp)-1; i>=0; --i) {
    if (sTemp[i] == '\\') {
      sTemp[i] = '\0';
      break;
    }
  }
  return string(sTemp);
}

#else 

std::string GetModulePath() {
  char sTemp[512];
  char *p = getcwd(sTemp, 512);
  return string(p);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <boost/algorithm/string.hpp>

#define INIBUFSIZE 4096

void ReformString(char *pFrom, char* pTo) {

  string s1(pFrom);
  boost::trim(s1);
  boost::trim_if(s1, std::bind2nd(std::equal_to<char>(), '\"'));  
  boost::trim_if(s1, std::bind2nd(std::equal_to<char>(), '\r'));  
  strcpy(pTo, s1.c_str());
}

//めんどいからバッファのサイズ調べてない
unsigned int GetPrivateProfileString(
  const char *szSection, const char *szKey, const char *szDefault, 
  char *szBuffer, unsigned int nBufferSize,
  const char *szIniFile) 
{
  ifstream ifs(szIniFile);
  if (!ifs) {
    cout << "file not found" << endl;
    strcpy(szBuffer, szDefault);
    return strlen(szBuffer);
  }
  std::string sLine;

  char szSection2[INIBUFSIZE];
  sprintf(szSection2, "[%s]", szSection);

  bool bSectionFound = false;
  while (getline(ifs, sLine)) {
    //空白，改行の残りかすを消す
    string sLine2 = boost::trim_copy_if(string(sLine.c_str()), boost::is_any_of(" \r"));

    //    cout << "Line2:" << sLine2 << " len: " << sLine2.size() << endl;
    //コメントまたは空行
    if ((sLine2.size() == 0) || (sLine2[0] == ';') || (sLine2[0] == '#')) continue;   

    if (bSectionFound) {
      if  (sLine2[0] == '[') break; //次のセクションが出てきた

      string::size_type nPos = sLine2.find('=');
      if (nPos == string::npos) continue; //=がない
      string sKey = boost::trim_copy_if(sLine2.substr(0,nPos), boost::is_any_of(" "));
      string sVal = boost::trim_copy_if(sLine2.substr(nPos+1), boost::is_any_of(" \""));
      if (sKey == string(szKey)) {
        strcpy(szBuffer, sVal.c_str());
        return sVal.size();
      }
      else {
        continue;
      }
    }
    else {
      if ( (sLine2[0] == '[') && (strcmp(szSection2, sLine2.c_str()) == 0) ) {
        bSectionFound = true;
      }
    }
  }

  strcpy(szBuffer, szDefault);
  return strlen(szBuffer);
}

unsigned int GetPrivateProfileInt(
  const char *szSection, const char *szKey, int nDefault, 
  const char *szIniFile) 
{
  char szBuffer[INIBUFSIZE];
  char szDefault[] = "";

  GetPrivateProfileString(szSection, szKey, szDefault, szBuffer, INIBUFSIZE, szIniFile);

  if (strcmp(szBuffer, szDefault)==0) {
    return (unsigned int)nDefault;
  }

  return atoi(szBuffer);
}

#include <stdio.h>
#include <termios.h>
//#include <term.h>
//#include <curses.h>
#include <unistd.h>

static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard()
{
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &initial_settings);
}

void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

class InitKeyBoard {
public:

  InitKeyBoard() {
    init_keyboard();
  }
  ~InitKeyBoard() {
    close_keyboard();
  }
};


int _kbhit()
{
    static InitKeyBoard kb;
    char ch;
    int nread;

    if(peek_character != -1)
        return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);

    if(nread == 1) {
        peek_character = ch;
        return 1;
    }
    return 0;
}


int _getch()
{
    char ch;
    if(peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    auto n = read(0, &ch, 1);
    if (n<0) {
      cout << "read error: " << errno << endl;
    }
    return (int)ch;
}   
void Sleep(unsigned long ms) {
  usleep(ms*1000);
}

#endif


double GetPrivateProfileDouble(const char* sModuleName, const char* pTarget, double dDefault,  const char* pFile) {
  string sDefault = boost::lexical_cast<string>(dDefault);
  char sBuf[512];
  GetPrivateProfileString(sModuleName, pTarget, sDefault.c_str(), sBuf, 512, pFile);
  return boost::lexical_cast<double>(sBuf);
}
