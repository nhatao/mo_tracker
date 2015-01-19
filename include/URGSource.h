#pragma once
#include "LRFSource.h"
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

class CURGSource :
  public CLRFSource
{
public:
  CURGSource(bool bTop=true);
  virtual ~CURGSource(void);

  bool Init(int nPort);

  virtual const unsigned short *ReadDataImpl() const {
    return _aCurrentRawData;
  }
  virtual void Measure();
  virtual void Reset();
  virtual bool CheckInvalidData(unsigned short nData) const {
//    if (nData<20) return false; else return true; //080304 なぜかエラーコードで100が帰ってくる
    if ((nData<101)||(nData=60000)) return false; else return true;
  }
  virtual double GetCurrentScanTime () const {return _dScanTime;}

  void RunThread();

protected:
  CSerialControl *_pSerialControl;
  boost::shared_ptr<boost::thread> _pDataObtainThread;
  boost::mutex _DataMutex;
  boost::condition _DataCondition;

  int SendURGMsg(const std::string& rsMsg);
  int ReadSerialUntilL2(char *pBuf, size_t nBufSize, double dTimeout);

  bool _bTop;
  mmtimer _ScanTimer;
  double _dScanTime;
  double _dScanTimeThread;
  unsigned char *_pData;
  unsigned short *_pCurrentData;

  char* _pBuf;
  size_t _nBufSize;
  std::string _sRequestMsg;
  int _nPortNo;

  volatile bool _bThreadFinished;
};
