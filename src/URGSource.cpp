#include "StdAfx_MOTracking.h"
#include "URGSource.h"
#include "SerialControl.h"

#include <sstream>
#include <iomanip>
#include <boost/bind.hpp>

using namespace std;

CURGSource::CURGSource(bool bTop)
{
  _bTop = bTop;
  _LRFProperty._bModeMM = true;

#ifdef _MSC_VER
  _pSerialControl = new CSerialControlWin();
#else 
  _pSerialControl = new CSerialControlUnix();
#endif
  _sLRFName = "URG";

  _aCurrentRawData = NULL;
  _pBuf = NULL;
  _pData = NULL;
  _pCurrentData = NULL;
  _nBufSize = 16000;
}

CURGSource::~CURGSource(void)
{

  if (_pDataObtainThread.get()) {
    _bThreadFinished = true;
    _pDataObtainThread->join();
  }

  if (_pSerialControl->IsOpened()) {
    SendURGMsg("QT\n");
  }
  delete _pSerialControl;
  delete[] _pCurrentData;
  delete[] _pData;
  delete[] _pBuf;
}

void CURGSource::Reset() {
}

bool CURGSource::Init(int nPort) {

  string sBDMsg;
  _pBuf = new char[_nBufSize+2000];

  if (_pSerialControl->Open(nPort,CSerialControl::BR115200, false)) {
    Sleep(10);
    _nPortNo = nPort;
    _pSerialControl->Flush();
    SendURGMsg("QT\n");
    _pSerialControl->Flush();
    if (!_bTop) {
      SendURGMsg("SCIP2.0\n");
    }
    SendURGMsg("BM\n");
    SendURGMsg("PP\n");
    istringstream sURGParam(_pBuf);
    string sParam;

//    cout << "Buf: " << sURGParam.str() << endl;

    map<string, int> mURGParams;
    mURGParams.clear();
    while (getline(sURGParam, sParam)) {
      //cout << sParam << endl;
      size_t p1 = sParam.find(':');
      size_t p2 = sParam.find(';');
      string sParamName = sParam.substr(0, p1);
      string sValue = sParam.substr(p1+1, p2-p1-1);
      mURGParams.insert(make_pair(sParamName, atoi(sValue.c_str())));
    }

    if(mURGParams.empty()){
      cout << "mURGParams is empty" << endl;
      return false;
    }

    int nARES = mURGParams.find("ARES")->second;
    int nAMIN = mURGParams.find("AMIN")->second;
    int nAMAX = mURGParams.find("AMAX")->second;
    int nAFRT = mURGParams.find("AFRT")->second;
    int nSCAN = mURGParams.find("SCAN")->second;

    _LRFProperty._dReso.set_deg(360.0/nARES);
    _LRFProperty._nElemNum = nAFRT*2+1;
    _LRFProperty._dFirstAngle.set_deg(-nAFRT*_LRFProperty._dReso.get_deg());
    _nFirstPos = nAMIN;
    _nLastPos = nAMAX+1;
    _LRFProperty._bModeMM = true;
    _LRFProperty._dScanTime = 1.0/(nSCAN/60.0);
    _LRFProperty._dTimeIncrement = _LRFProperty._dScanTime/nARES;
    _LRFProperty._dMinRange = 0;
    _LRFProperty._dMaxRange = DBL_MAX;
    _aCurrentRawData = new unsigned short[_LRFProperty._nElemNum];
    _pCurrentData = new unsigned short[_LRFProperty._nElemNum];

    memset(_aCurrentRawData, 0, _LRFProperty._nElemNum*sizeof(unsigned short)); 
    _pData = new unsigned char[_LRFProperty._nElemNum*3];
    stringstream ss;
    ss << "MD0000" << std::setw(4) << std::setfill('0') << _LRFProperty._nElemNum-1 << "00000\n"; //無限
    _sRequestMsg = ss.str();
  }
  else {
    ESStreamException ss; ss << "URG Initialize Failed!! Port " << nPort;
    throw ss;
  }

  InitArray();

  _pDataObtainThread = boost::shared_ptr<boost::thread>(
    new boost::thread(boost::bind(&CURGSource::RunThread, this)));

  Sleep(100);
  return true;
}


int CURGSource::ReadSerialUntilL2(char *pBuf, size_t nBufSize2, double dTimeout) {

  unsigned char *pCurPos = (unsigned char*)pBuf;
  size_t nRead=0;
  mmtimer mt;
  size_t nBufSize = nBufSize2-1; //最後の'\0'用
  do {
    int nReadTemp = _pSerialControl->ReadData(pCurPos, nBufSize-nRead);
    nRead+=nReadTemp;
    pCurPos+=nReadTemp;
    if ( (nRead > 2) && (pBuf[nRead-1] == '\n') && (pBuf[nRead-2] == '\n') ) {
      pBuf[nRead] = '\0';
      return nRead;
    }
    Sleep(1);
  } while ((mt.elapsed() < dTimeout) || (nRead < nBufSize) );

  pBuf[nBufSize2-1] = '\0';
  ESStreamException ess;
  ess << "ReadSerialUntilL2 Timeout!" << " Port: " << _nPortNo << " Res: " << pBuf << " Size: " << nRead;
  throw ess;
}

int CURGSource::SendURGMsg(const std::string& rsMsg) {
  size_t nSendSize = rsMsg.size();
  if (rsMsg[rsMsg.size()-1] != '\n') {//最後が改行で終わってない場合、改行を付ける
#ifdef _MSC_VER
    sprintf_s(_pBuf, _nBufSize, "%s\n", rsMsg.c_str());
#else
    sprintf(_pBuf, "%s\n", rsMsg.c_str());
#endif
    _pSerialControl->WriteData((const unsigned char *)_pBuf, strlen(_pBuf));
    ++nSendSize;
  }
  else {
    _pSerialControl->WriteData((const unsigned char *)rsMsg.c_str(), rsMsg.size());
  }
  int nRecvSize = ReadSerialUntilL2(_pBuf, _nBufSize, 0.8);

  std::string sRes = "00";
  sRes[0] = _pBuf[nSendSize];
  sRes[1] = _pBuf[nSendSize+1];

  int nRes;
  try {
    nRes = boost::lexical_cast<int>(sRes);
    return nRes;
  }
  catch (std::exception &e) {
    ESStreamException ess; 
    ess << e.what() << "\n";
    ess << "Send/RecvSize: " << nSendSize << "/" << nRecvSize << "\n";
    ess << "sRes: " << sRes << "\n";
    ess << "pBuf: " << _pBuf << "\n";
  }
  return  0;
}

#include "OutputDebugStream.h"
//複数回がまとまってきた場合，最後のデータの先頭ポインタを返す
char* GetLatestDataPos(char* pBuf, size_t nSize) {

  if (nSize <= 3) return pBuf;
  int nLN2Num = 0;
  size_t nPos = 0;
  for (size_t i=nSize-2; i!=0; --i) {
    if ((pBuf[i] == '\n') && (pBuf[i+1] == '\n')) {
      ++nLN2Num;
      nPos = i+2;
      if (nLN2Num==2) break;
    }
  }

  if (nLN2Num <= 1) return pBuf;
  else {
    return pBuf+nPos;
  }
}

void CURGSource::RunThread() {

  _bThreadFinished = false;

  try {
    _pSerialControl->WriteData((const unsigned char *)_sRequestMsg.c_str(), _sRequestMsg.size());
    _ScanTimer.restart();
    Sleep(10);
    int nObtainNum = 0;
    while (!_bThreadFinished) {
      int nReplyNum = ReadSerialUntilL2(_pBuf, _nBufSize, 1.0);
//      dout << "ReplyURG: " << nReplyNum << endl;
      double dTime = _ScanTimer.elapsed();
//      char* pCurPos = _pBuf;
      char* pCurPos = GetLatestDataPos(_pBuf, nReplyNum);
      int nLine = 0;
      int nDataPos = 0;
      while (true) {
        char* pTemp = pCurPos;
        while(*pTemp != '\n') ++pTemp;
        size_t nLen = pTemp-pCurPos;
        if (nLen == 0) break;
        if (nLine >= 3) {
          memcpy(_pData+nDataPos, pCurPos, nLen-1);
          nDataPos+=(nLen-1);
        }
        pCurPos+=(nLen+1);
        if (nReplyNum <= (pCurPos-_pBuf)) break;
        ++nLine;
      }
      int nTotal = 0;
      _dScanTimeThread = dTime;
      {
        boost::mutex::scoped_lock lk(_DataMutex);
        for (size_t i=0; i<_LRFProperty._nElemNum; ++i) {
          _pCurrentData[i] = 4096*(_pData[i*3]-0x30) + 64*(_pData[i*3+1]-0x30) + (_pData[i*3+2]-0x30);
          nTotal+=_pCurrentData[i];
        }
        _DataCondition.notify_all();
      }
      if ((nObtainNum % 100) == 1) {
        cout << "URG FPS: " << nObtainNum/dTime << endl;
      }
    }
    boost::mutex::scoped_lock lk(_DataMutex);
    _DataCondition.notify_all();
  }
  catch (std::exception &e) {
    _bThreadFinished = true;
    cout << "CURGSource::RunThread exception: " << e.what() << endl;
  }
}

void CURGSource::Measure() {
  boost::mutex::scoped_lock lk(_DataMutex);
  boost::xtime xt;
  boost::xtime_get(&xt, boost::TIME_UTC_);
  xt.sec += 1;
  mmtimer mt;
  if( _DataCondition.timed_wait( lk, xt ) ) {
    _dScanTime = _dScanTimeThread;
    memcpy(_aCurrentRawData, _pCurrentData, _LRFProperty._nElemNum*sizeof(unsigned short));
  }
  else {
    throw std::logic_error("CURGSource::Measure() Timeout!");
  }
}
