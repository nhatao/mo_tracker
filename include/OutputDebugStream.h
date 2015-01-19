#pragma once
#include <iostream>
#include "mt_ofstream.h"
#include <boost/thread.hpp>

#ifdef _MSC_VER

#include <windows.h>
#undef max
#undef min
#endif

//windows 0:非表示 1:OutputDebugStream  2:printf 3:ファイル
//Linux   0:非表示 1:ファイル(dout.txt) 2:printf 3:ファイル
template <class Ch,class Tr=std::char_traits<Ch> >
class basic_outputdebug_streambuf : public std::basic_streambuf<Ch,Tr> {
public:
  basic_outputdebug_streambuf(void){
    _nMode = 1;
  }
  ~basic_outputdebug_streambuf(void){
    if (_file.is_open()) _file << std::endl;
  }

  void SetMode(int n) {
#ifdef _MSC_VER
    if (n==3) {
#else
    if ((n==3) || (n==1)) {
#endif
      SetFileName();
      return;
    }
    if ((n < 0) || (n > 3)) {
      throw std::logic_error("dout Invalid SetMode");
    }
    _nMode = n;
  }
  void SetFileName(const std::string &rsFileName="") {
    if (!_file.is_open()) {
      std::string s="dout.txt";
      if (rsFileName != "") s=rsFileName;
      _file.open(s.c_str(), std::ios::out | std::ios::app);
    }
    _nMode = 3;
  }
  int GetMode() const {
    return _nMode;
  }

protected:
  int overflow( int nCh = EOF ) {
    _mutex.lock();
    _buffer.append(1,(char)nCh);
    _mutex.unlock();
    return 0;
  }
  int sync() {

    _mutex.lock();
    if (_nMode == 1) {
#ifdef _MSC_VER
      OutputDebugString(_buffer.c_str());
#else
      printf("%s", _buffer.c_str());
#endif
    }
    else if (_nMode == 2) {
//      printf(_buffer.c_str());
      printf("%s", _buffer.c_str());
    }
    else if (_nMode == 3) {
      _file << _buffer;
    }

    _buffer.clear();
    _mutex.unlock();
    return 0;
  }
private:
  int _nMode;
  std::string _buffer;
  std::ofstream _file;

  boost::mutex _mutex;
};


template <class Ch,class Tr=std::char_traits<Ch> >
class basic_outputdebug_stream : public std::basic_ostream<Ch,Tr>{

  typedef std::basic_ostream<Ch,Tr> P;
public:
  ~basic_outputdebug_stream(void){}
  static basic_outputdebug_stream & getInstance() {
    static basic_outputdebug_stream singleton;
    return singleton;
  }
  void SetMode(int nMode) {
    ((basic_outputdebug_streambuf<Ch,Tr>*)(P::rdbuf()))->SetMode(nMode);
  }
  int GetMode() const {
    return ((const basic_outputdebug_streambuf<Ch,Tr>*)(P::rdbuf()))->GetMode();
  }
private:
  basic_outputdebug_stream(void)
    : std::basic_ostream<Ch,Tr>(new basic_outputdebug_streambuf<Ch,Tr>()) {}
};

typedef basic_outputdebug_streambuf<char> outputdebug_streambuf;
typedef basic_outputdebug_stream<char> outputdebug_stream;

#define dout (outputdebug_stream::getInstance())
