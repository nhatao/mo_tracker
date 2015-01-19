#include "StdAfx_MOTracking.h"
#include "mmtimer.h"

#ifdef _MSC_VER
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

mmtimer::mmtimer(void)
  {
    QueryPerformanceFrequency( (LARGE_INTEGER *)&_freq );
    QueryPerformanceCounter( (LARGE_INTEGER *)&_start_time);
  }

mmtimer::~mmtimer(void)
  {
  }

double mmtimer::elapsed() {
  LONGLONG temp;
  QueryPerformanceCounter( (LARGE_INTEGER *)&temp);
  return ((double)(temp-_start_time) / _freq );
}

void mmtimer::restart() {
  QueryPerformanceCounter( (LARGE_INTEGER *)&_start_time);
}
#endif
