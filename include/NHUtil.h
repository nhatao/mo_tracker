#pragma once

#include <string>

#ifdef _MSC_VER

#include <conio.h>
const int g_nEnterCode = 13;

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef min
#undef max

#else 

const int g_nEnterCode = 10;
unsigned int GetPrivateProfileString(
  const char *szSection, const char *szKey, const char *szDefault, 
  char *szBuffer, unsigned int nBufferSize,
  const char *szIniFile);

unsigned int GetPrivateProfileInt(
  const char *szSection, const char *szKey, int nDefault, 
  const char *szIniFile);


int _kbhit();
int _getch();
void Sleep(unsigned long ms);

#endif

std::string GetModulePath();
double GetPrivateProfileDouble(const char* sModuleName, const char* pTarget, double dDefault, const char* pFile);

#ifndef _MSC_VER
#define _isnan std::isnan
#endif
