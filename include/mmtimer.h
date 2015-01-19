#pragma once

#ifdef _MSC_VER

class mmtimer
{
public:

  mmtimer(void);
  virtual ~mmtimer(void);
  double elapsed();
  void restart();

private:

  _int64 _start_time;
  _int64 _freq;
};

#else

#include <sys/time.h> 

class mmtimer
{
public:

  mmtimer(void) {
    restart();
  }
  virtual ~mmtimer(void) {}

  double elapsed() {
    timeval tv2;
    gettimeofday(&tv2, NULL);
    double time2 = tv2.tv_sec + (double)tv2.tv_usec*1e-6;
    return time2-_time;
    
  }
  void restart() {
    timeval tv;
    gettimeofday(&tv, NULL);
    _time = tv.tv_sec + (double)tv.tv_usec*1e-6;
  }

private:

  double _time;

};

#endif

#include <float.h>
#include <iostream>

class CTimerSet {
public:

  CTimerSet(int n) {
    _adAveTime = NULL;
    _adMinTime = NULL;
    _adMaxTime = NULL;
    Init(n);
  }
  virtual ~CTimerSet() {
    delete[] _adAveTime;
    delete[] _adMinTime;
    delete[] _adMaxTime;
  }
  void Init(int n) {
    delete[] _adAveTime;
    delete[] _adMinTime;
    delete[] _adMaxTime;
    _adAveTime = new double[n];
    _adMinTime = new double[n];
    _adMaxTime = new double[n];
    for (int i=0; i<n; ++i) {
      _adAveTime[i] = 0;
      _adMinTime[i] = DBL_MAX;
      _adMaxTime[i] = -DBL_MAX;
    }
    _nFrame = 0;
    _timer.restart();
  }
  void SetTime(int n) {
    double d = _timer.elapsed();
    _adAveTime[n] += d;
    if (_adMaxTime[n] < d) _adMaxTime[n] = d;
    if (_adMinTime[n] > d) _adMinTime[n] = d;
    _timer.restart();
  }
  void IncCnt() {
    ++_nFrame;
  }
  void Show(int n, std::ostream& strm = std::cout) {
    strm << _adMinTime[n] << " / " << _adAveTime[n]/_nFrame << " / " << _adMaxTime[n] << " ";
  }
  int GetFrame() {return _nFrame;}

private:

  double* _adAveTime;
  double* _adMinTime;
  double* _adMaxTime;
  mmtimer _timer;
  int _nFrame;
};
