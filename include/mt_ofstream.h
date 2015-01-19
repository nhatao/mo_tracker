#pragma once

#include <ostream>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/ref.hpp>
#include <boost/shared_ptr.hpp>
#ifdef _MSC_VER
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <unistd.h>
#endif

//書き込み時の待ち時間でリアルタイム性がなくなるのを防ぐため、書き込みをスレッド化したofstream
//sleepはmsec
class basic_multithread_ofstreambuf_worker {
public:

  basic_multithread_ofstreambuf_worker(std::ofstream *pofs, int nSleepTime=50) {
    _pofs = pofs;
    _nSleepTime = nSleepTime;
    _bEnd = false;
  }
  void CopyStr(const char* pStr) {    
    lock lk(_mutex);
    _sStr1 << pStr;
  }
  
  void operator() (){
    std::string sBuf;
    while (true) {
#ifdef _MSC_VER
      Sleep(_nSleepTime);
#else
      usleep(1000*_nSleepTime);
#endif
      {
        lock lk(_mutex);
        sBuf = _sStr1.str();
        _sStr1.str("");
        _sStr1.clear();
      }
      if (!sBuf.empty()) {
        (*_pofs) << sBuf;
      }
      if (_bEnd) {
        break;
      }
    }
  }

  volatile bool _bEnd;

protected:
  basic_multithread_ofstreambuf_worker(basic_multithread_ofstreambuf_worker &r);
  basic_multithread_ofstreambuf_worker& operator=(basic_multithread_ofstreambuf_worker &r);

  std::ostringstream _sStr1;
  std::ofstream *_pofs;
  int _nSleepTime;
  typedef boost::mutex::scoped_lock lock;
  boost::mutex _mutex;
};

template <typename Ch_T, typename Tr_T = std::char_traits<Ch_T> >
class basic_multithread_ofstreambuf: public std::basic_stringbuf<Ch_T, Tr_T>
{
public:

 typedef std::basic_stringbuf<Ch_T, Tr_T> _P;

  basic_multithread_ofstreambuf(): std::basic_stringbuf<Ch_T, Tr_T>(),
                                   _Worker(&_ofs, 1)
 {
 }

 virtual ~basic_multithread_ofstreambuf() {
   close();
 }

 std::ofstream &ofs() {return _ofs;}
 const std::ofstream &ofs() const {return _ofs;}

 void open(const char* pFileName, std::ios::openmode Mode = std::ios::out) {
   close();
   _ofs.open(pFileName, Mode);
   _pWriteThr = boost::shared_ptr<boost::thread>(new boost::thread(boost::ref(_Worker)));
 }
 void close() {
   if (_pWriteThr) {
     sync();
     _Worker._bEnd = true;
     _pWriteThr->join();
   }
   _pWriteThr.reset();
   _Worker._bEnd = false;
   _ofs.close();
   _ofs.clear();
 }

 
protected:
 int sync(void) {

   *_P::pptr() = '\0';
   _Worker.CopyStr(_P::pbase());
   _P::pbump((int)(_P::pbase() - _P::pptr()));
   return 0;
 }
 
 std::ofstream _ofs;
 boost::shared_ptr<boost::thread> _pWriteThr;
 basic_multithread_ofstreambuf_worker _Worker;
};


template <typename Ch_T, typename Tr_T = std::char_traits<Ch_T> >
class basic_multithread_ofstream: public std::basic_ostream<Ch_T, Tr_T>
{
public:
  basic_multithread_ofstream(const char* pFileName = NULL, std::ios::openmode Mode = std::ios::out)
   : std::basic_ostream<Ch_T, Tr_T>(new basic_multithread_ofstreambuf<Ch_T, Tr_T>())
 {
   if (pFileName) {
     open(pFileName,Mode);
   }
 }

 virtual ~basic_multithread_ofstream()
 {
   std::basic_ostream<Ch_T, Tr_T>::flush();
   delete std::basic_ostream<Ch_T, Tr_T>::rdbuf();
 }
 void close() {
   ((basic_multithread_ofstreambuf<Ch_T, Tr_T>*)(std::basic_ostream<Ch_T, Tr_T>::rdbuf()))->close();
 }
 void clear() {
   std::basic_ostream<Ch_T, Tr_T>::clear();
   ofs().clear();
 }
 void open(const char* pFileName, std::ios::openmode Mode = std::ios::out) {
   ((basic_multithread_ofstreambuf<Ch_T, Tr_T>*)(std::basic_ostream<Ch_T, Tr_T>::rdbuf()))->open(pFileName, Mode);
 }

 bool is_open() const {
   return ofs().is_open();
 }


protected:

  std::ofstream &ofs() {return ((basic_multithread_ofstreambuf<Ch_T, Tr_T>*)(std::basic_ostream<Ch_T, Tr_T>::rdbuf()))->ofs();}
  const std::ofstream &ofs() const {return ((basic_multithread_ofstreambuf<Ch_T, Tr_T>*)(std::basic_ostream<Ch_T, Tr_T>::rdbuf()))->ofs();}


};

typedef basic_multithread_ofstreambuf<char>  mt_streambuf;
typedef basic_multithread_ofstream<char>  mt_ofstream;
//typedef basic_multithread_ofstreambuf<wchar_t>  mt_wstreambuf;
//typedef basic_multithread_ofstream<wchar_t>  mt_wofstream;
