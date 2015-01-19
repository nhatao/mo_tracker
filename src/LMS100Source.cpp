
#include "StdAfx_MOTracking.h"

#include "LMS100Source.h"
#ifdef _MSC_VER
#include "SocketWin.h"
#else
#include "SocketUnix.h"
#endif
#include "ColorCout.h"

using namespace std;

CLMS100Source::CLMS100Source(bool bShowDebugMsg) : CLRFSource() 
{
  _nReadBufSize = 1024*100;
  _pReadBuf = new char[_nReadBufSize];
  _bInitialized = false;
  _bReaderRunning = false;

  _LRFProperty._dFirstAngle.set_deg(-45);
  _dEndDeg.set_deg(225);
  _LRFProperty._dReso.set_deg(0.25);
  _nQueueSizeMax = 100;

  _LRFProperty._bModeMM = true;
  _sLRFName = "LMS100";
  _nInterval = 0;
  _LRFProperty._bInverse = false;

  _bShowDebugMsg = bShowDebugMsg;
}

CLMS100Source::~CLMS100Source(void)
{

  if (_pReader.get()) {
    SendLMSCommand("sEN LMDscandata 0"); //垂れ流しモード終了
    _bReaderRunning = false;
    _pReader->join();
  }

  delete [] _pReadBuf;
}

int ParseResponse1(const std::string &rS1) {

  int nCnt = 0;
  for (const char *p=rS1.c_str(); (*p) != '\0'; ++p) {
    if (*p == ' ') {
      ++nCnt;
      if (nCnt == 2) {
        return (int)strtoul(p+1, NULL, 16);
      }
    }
  }
  ESStreamException ess;
  ess << __FUNCTION__  << " failed: " << rS1;
  return 0;
}


void ParseResponse(const std::string &rS1, int nResNum, vector<int> &rsResult) {

  int nCnt = 0;
  for (const char *p=rS1.c_str(); (*p) != '\0'; ++p) {
    if (*p == ' ') {
      ++nCnt;
      if (nCnt >= 2) {
        int n2  = (int)strtoul(p+1, NULL, 16);
        rsResult.push_back(n2);
        if (rsResult.size() == nResNum) return;
      }
    }
  }
  ESStreamException ess;
  ess << __FUNCTION__  << " failed: " << rS1;
}

void CLMS100Source::Init(const std::string &rsIP, int nPort, EFreqAndAngleMode eMode, double dStartAngle, double dEndAngle) {

  if (_bInitialized) {
    //既に開いていたら一旦閉じて開き直す方がいい？
    throw std::logic_error("LMS100 Already Opened!");
  }

  _bInitialized = false;
  _pSocket.reset();
#ifdef _MSC_VER
  _pSocket = boost::shared_ptr<CSocketClient>(new CSocketClientWin());
#else
  _pSocket = boost::shared_ptr<CSocketClient>(new CSocketClientUnix());
#endif
  _pSocket->SetupSocket(rsIP, nPort, true);

  if (_pReader) {
    _bReaderRunning = false;
    _pReader->join();
  }
  _pReader = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CLMS100Source::ReadDataThread, this)));
  Sleep(100);
  string sRes;
  int nRes;
  nRes = ParseResponse1(SendLMSCommand("sMN SetAccessMode 3 F4724744")); //変更許可
  if (nRes == 0) {
    throw std::logic_error("SetAccessMode Failed!");
  }

  SetNto1FilterMode(true);
  SetFogFilterMode(false);
  SetMeanFilterMode(false);
  InitStatus(eMode, dStartAngle, dEndAngle);
  SendLMSCommand("sMN Run");
  SendLMSCommand("sEN LMDscandata 1"); //垂れ流しモード開始

  //計測可能状態になるまで待機
  mmtimer mt;
  while (true) {
    string sRes = SendLMSCommand("sRN SCdevicestate");
    if (sRes == "sRA SCdevicestate 1") {
      break;
    }
    else {
      cout << "Waiting LMS Response" << endl;
      Sleep(1000);
    }
  }

  if (_bShowDebugMsg) {
    cout << "LMS100 Status" << endl;
    cout << "LRFPointNum=" << _LRFProperty._nElemNum << endl;
    cout << "UpdateTime=" << _nUpdateTime << endl;
    cout << "Resolution=" << _LRFProperty._dReso.get_deg() << endl;
    cout << "FirstDeg=" << _LRFProperty._dFirstAngle.get_deg() << endl;
    cout << "LastDeg=" << _dEndDeg.get_deg() << endl;
  }

  _sIP = rsIP;
  _nPort = nPort;

  _nLastScanCounter = 0;
  _bInitialized = true;
}

std::pair<int, double> CLMS100Source::GetFreqAndResoFromMode(EFreqAndAngleMode eMode) {
  if (eMode == e025DegAt25Hz) return make_pair(25, 0.25);
  else if (eMode == e050DegAt25Hz) return make_pair(25, 0.50);
  else if (eMode == e050DegAt50Hz) return make_pair(50, 0.50);
  else {
    ESStreamException ess;
    ess << __FUNCTION__ << " Unknown Mode: " << (int)eMode;
    throw ess;
  }
}

#include <boost/tuple/tuple.hpp>

void CLMS100Source::InitStatus(EFreqAndAngleMode eMode, double dStartAngle, double dEndAngle) {

  std::vector<int> vScanCfgRes;
  ParseResponse(SendLMSCommand("sRN LMPscancfg"), 5, vScanCfgRes);

  int nCurrentHz = (int)round(vScanCfgRes[0]/100.0);
  double dCurrentReso = vScanCfgRes[2]/10000.0;
  cout << "Current Hz=" << nCurrentHz << ", Current Reso=" << dCurrentReso << endl;

  int nHz;
  double dReso;
  boost::tie(nHz, dReso) = GetFreqAndResoFromMode(eMode);
  cout << "Desired Hz=" << nHz << ", Desired Reso=" << dReso << endl;

  _nUpdateTime = 1000/nCurrentHz;

  if  ((nCurrentHz != nHz) || (dReso != dCurrentReso)) {
    SetFreqAndResolution(eMode);
  }
  SetScanDataAngle(dStartAngle, dEndAngle); 

  _LRFProperty._dReso.set_deg(dReso);
}

void CLMS100Source::StartMeasure() {
  //do nothing
}

bool CLMS100Source::NeedToWait() {
  boost::recursive_mutex::scoped_lock lk2(_HistoryMutex);
  if (_qLMSHistory.empty()) return true;

  unsigned short nCurScanCounter = _qLMSHistory.back()->nScanCounter;
  if (nCurScanCounter != _nLastScanCounter) {
    return false;
  }
  return true;
}

void CLMS100Source::RequestData() { }
void CLMS100Source::Measure() {

  boost::recursive_mutex::scoped_lock lk2(_HistoryMutex);
  if (NeedToWait()) {
    boost::xtime xt;
    boost::xtime_get(&xt, boost::TIME_UTC_);
    xt.sec += 10;
    if (!_HistoryWaitSet.timed_wait( lk2, xt )) {
      ESStreamException ess; ess << __FUNCTION__ << "TimeOut!";
      throw ess;
    }
  }
  const auto &p = _qLMSHistory.back();
  const auto &vData = p->LaserData1.vData;    
  if (vData.size() != _LRFProperty._nElemNum) {
    ESStreamException ess; ess << __FUNCTION__ << "Unknown Size Mismatch: Data=" << vData.size() << ", Desired=" << _LRFProperty._nElemNum;
    throw ess;
  }
  for (size_t i=0; i<vData.size(); ++i) {
    _aCurrentRawData[i] = vData[i];
  }
  _nLastScanCounter = p->nScanCounter;
}

void CLMS100Source::Reset() {
  //do nothing
}

bool CLMS100Source::CheckInvalidData(unsigned short nData) const {
  
  return true;
}

string CLMS100Source::SendLMSCommand(const std::string &rsMsg, double dTimeOut) {

  ostringstream iss; iss << (char)0x02 << rsMsg << (char)0x03 ;
  size_t nLen = iss.str().length();
  int n = _pSocket->SendMsg(iss.str().c_str(), iss.str().length());
  if (n != (int)nLen) {
    ESStreamException ess;
    ess << __FUNCTION__ << " data size NG: StringLen = " << nLen << " Reply = " << n;
    throw ess;
  }

  if (_pReader) {
    boost::recursive_mutex::scoped_lock lk2(_ReplyMutex);
    boost::xtime xt;
    boost::xtime_get(&xt, boost::TIME_UTC_);
    double dSec = floor(dTimeOut);
    xt.sec += (int)dSec;
    xt.nsec += (int)round((dTimeOut-dSec)*1000*1000);
    mmtimer mt;
    if( _ReplyWaitSet.timed_wait( lk2, xt ) ) {
      if (_bShowDebugMsg) {
        ccout->SetColor(ColorCout::eGreen);
        ccout << "Message: " << rsMsg << endl;
        ccout << "Reply:   " << _sReply << endl;
      }


      return _sReply;
    }
    else {
      ESStreamException ess; ess << "Timeout!! Msg: " << rsMsg;
      throw ess;
    }
  }
  else {
    cout << "_pReader not initialized" << endl;
  }
  return "";
}


void SLMS100ScanData::SLaserData::ReadFromMsg(std::istream &iss) {

  unsigned short nDataNum;
  unsigned int n1, n2, n3;
  iss >> n1 >> n2 >> n3;
  iss >> nSteps >> nDataNum;
  memcpy(&dScaleFactor, &n1, 4);
  memcpy(&dScaleFactorOffset, &n2, 4);
  memcpy(&nStartAngle, &n3, 4);

//    cout << "Scale: " << pTarget->dScaleFactor << " SAngle: " << pTarget->nStartAngle/10000.0 << " Step: " << pTarget->nSteps/10000.0 << " DataNum: " << nDataNum << endl;

  //整合性チェック
  if ( ((nSteps != 5000) && (nSteps != 2500)) || (dScaleFactor != 1) ) {
    ESStreamException ess;
    ess << "LMSData Something Wrong: " << " Step=" << nSteps << " ScaleFactor=" << dScaleFactor;
    throw ess;
  }

//  cout << "Step=" << nSteps << " ElemNum=" << nDataNum << endl;

  unsigned short nD;
  for (unsigned short i=0; i<nDataNum; ++i) {
    iss >> nD;
    vData.push_back(nD);
  }
  bEnable = true;
}

void SLMS100ScanData::ReadFromMsg(const char* pMsg) {

  Init();
  istringstream iss(pMsg);
  iss >> setbase(16);

  string sType, sCommand;
  iss >> sType >> sCommand >> nVersion >> nDeviceID >> nSerialNo >> nDeviceStatus1 >> nDeviceStatus2;
  iss >> nMsgCounter >> nScanCounter >> nPowerUpDuration >> nTransmissionDuration >> nInputStatus1 >> nInputStatus2  >> nOutputStatus1 >> nOutputStatus2 >> nReserved;
  unsigned int nScanningFrequency, nMeasurementFrequency;
  iss >> nScanningFrequency >> nMeasurementFrequency;
  //Encoder Read
  unsigned short nAmountofEncoder;
  unsigned short n1, n2;
  iss >> nAmountofEncoder;
  for (unsigned short n=0; n<nAmountofEncoder; ++n) {
    iss >> n1 >> n2;
    vEncoderData.push_back(make_pair(n1, n2));
  }

  //16bit Data Read
  unsigned short nAmountof16BitChannel;
  iss >> nAmountof16BitChannel;

  if (nAmountof16BitChannel > 4) {
    ccout << "Error: nAmountof16BitChannel = " << nAmountof16BitChannel << endl;
    return;
  }
  for (unsigned short n=0; n<nAmountof16BitChannel; ++n) {
    string sType;
    iss >> sType; 
    SLaserData *pTarget;
    if (sType == "DIST1") pTarget = &LaserData1;
    else if (sType == "DIST2") pTarget = &LaserData2;
    else if (sType == "RSSI1") pTarget = &Remission1;
    else if (sType == "RSSI2") pTarget = &Remission2;
    else {
      ccout << "16Bit Unknown Type: " << sType << endl;
      continue;
    }
    pTarget->ReadFromMsg(iss);
  }

  //8bit Data Read
//  cout <<"16Bit: " << nAmountof16BitChannel << endl;
  unsigned short nAmountof8BitChannel;
  iss >> nAmountof8BitChannel;
  if (nAmountof8BitChannel > 4) {
    ccout << "Error: nAmountof8BitChannel = " << nAmountof8BitChannel << endl;

    ostringstream sFileName; sFileName << "data" << nMsgCounter << ".dat";

    ofstream ofs(sFileName.str().c_str());
    ofs << pMsg << endl;

    ESStreamException ess; ess << "Error at: " << nMsgCounter;
    throw ess;
    return;
  }
  for (unsigned short n=0; n<nAmountof8BitChannel; ++n) {
    string sType;
    iss >> sType; 
    SLaserData *pTarget;
    if (sType == "RSSI1") pTarget = &Remission1;
    if (sType == "RSSI2") pTarget = &Remission2;
    else {
      ccout << "8Bit Unknown Type: " << sType << endl;
      continue;
    }
    pTarget->ReadFromMsg(iss);
  }

  //Position Data
  iss >> bHasPosition;
  if (bHasPosition) {
    unsigned int aN[6];
    for (int i=0; i<6; ++i) iss >> aN[i];
    memcpy(&dX, &aN[0], 4);
    memcpy(&dY, &aN[1], 4);
    memcpy(&dZ, &aN[2], 4);
    memcpy(&dXR, &aN[3], 4);
    memcpy(&dYR, &aN[4], 4);
    memcpy(&dZR, &aN[5], 4);
    iss >> nKindofRotation;
  }

  iss >> bHasDeviceName;
  if (bHasDeviceName) iss >> sDeviceName;

  iss >> bHasComment;
  if (bHasComment) iss >> sComment;

  iss >> bHasTime;
  if (bHasTime) {
    iss >> nYear >> nMonth >> nDay >> nHour >> nMinute >> nSecond >> nMicroSecond;
  }

  iss >> bHasEvent;
  if (bHasEvent) {
    iss >> sType >> nEncoderPositionEvent >> nTimeofEvent;
    unsigned int n;
    iss >> n; memcpy(&nAngleofEvent, &n, 4);
  }
}


void CLMS100Source::InterpretScanData(const char* pMsg) {

  static int nErrorNo = 0;
  static mmtimer mt;
  static int nCount = 0;

  boost::shared_ptr<SLMS100ScanData> pData(new SLMS100ScanData());
  pData->ReadFromMsg(pMsg);

  if (_bShowDebugMsg) {
    ++nCount;
    if (nCount%50 == 0) {
      ccout->SetColor(ColorCout::eCyan);
      ccout << "FPS=" << nCount/mt.elapsed() << endl;
      nCount = 0;
      mt.restart();
    }
  }
  {
    boost::recursive_mutex::scoped_lock lk(_HistoryMutex);
    if (_qLMSHistory.size() >= _nQueueSizeMax) {
      _qLMSHistory.pop();
    }
    _qLMSHistory.push(pData);
    _HistoryWaitSet.notify_all();
  }
}


void CLMS100Source::InterpretResponseData(const char* pMsg) {
  ccout->SetColor(ColorCout::eGreen);
  {
    boost::recursive_mutex::scoped_lock lk(_ReplyMutex);
    _sReply = pMsg;
    _ReplyWaitSet.notify_all();
  }
}

void CLMS100Source::InterpretMsg(const char* pMsg) {

  ccout->SetColor(ColorCout::eRed);

  if (pMsg[0] != 0x02) {
    ccout << __FUNCTION__ <<  " Something Wrong: FirstByte=" << (int)pMsg[0]  << "/" << pMsg[0] << endl;
    ccout << pMsg << endl;
    InterpretScanData(pMsg); //temp
    return;
  }

  char str1[] = "sRA LMDscandata";
  char str2[] = "sSN LMDscandata";

  if ( (memcmp(pMsg+1, str1, strlen(str1)) == 0) || (memcmp(pMsg+1, str2, strlen(str2)) == 0) ) {
    InterpretScanData(pMsg+1);
  }
  else {
    InterpretResponseData(pMsg+1);
  }
}

//Delimを列挙してrvDelimPosに入れる．最後がDelimで終わっていなければその分をpOddBufに代入し，nOddSizeにそのバイト数を入れる
void ObtainData(const char* pBuf, int nRecvSize, std::vector<int> &rvDelimPos, char *pOddBuf, int &nOddSize) {


  rvDelimPos.clear();
  for (int i=0; i<nRecvSize; ++i) {
    if ( *(pBuf+i) == (char)0x03) {
      rvDelimPos.push_back(i);
    }
  }
  int nLastPos = -1;
  if (rvDelimPos.empty()) {
    nOddSize = nRecvSize;
  }
  else {
    nLastPos = rvDelimPos.back();
    nOddSize = nRecvSize-nLastPos-1;
  }
  if (nOddSize > 0) {
    memcpy(pOddBuf, pBuf+nLastPos+1, nOddSize);
  } 
}

#include "mt_ofstream.h"

void CLMS100Source::ReadDataThread() {


#ifdef _MSC_VER
    HANDLE hThread = GetCurrentThread();
    SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
#endif

  _bReaderRunning = true;
  int nBufSize = 1024*100;
  char *pBuf1 = new char[nBufSize+1];
  char *pBuf2 = new char[nBufSize+1];
//  char *pResult = new char[nBufSize+1];
  char *pBuf = pBuf1;
  char *pBufReserve = pBuf2;

  int nCurPos = 0;
  int nFirstPos = 0;

  vector<int> nvTemp;
  ostringstream oss;
  int nC = 0;

  while (_bReaderRunning) {

    try {
      int nRecv = 0;
      {
        boost::recursive_mutex::scoped_lock lk(_SocketMutex);
        nRecv = _pSocket->ReceiveMsg(pBuf+nFirstPos, nBufSize-nFirstPos);
      }
      if (nRecv>0) {
        pBuf[nFirstPos+nRecv] = '\0';
        int nOddSize = 0;
        vector<int> vDelimPos;
        ObtainData(pBuf, nRecv+nFirstPos, vDelimPos, pBufReserve, nOddSize);

        if (!vDelimPos.empty())  {
          for (size_t i=0; i<vDelimPos.size(); ++i) {
            pBuf[vDelimPos[i]] = '\0';
          }
          for (int i=-1; i<(int)(vDelimPos.size())-1; ++i) {
            size_t nPos = 0;
            if (i>=0) nPos = vDelimPos[i]+1;
            InterpretMsg(pBuf+nPos); 
          }
        }
        std::swap(pBuf, pBufReserve);
        nFirstPos = nOddSize;
        Sleep(0);
      }
      else {
        Sleep(1);
      }
    }
    catch (std::exception &e) {
      ccout << "Exception in " << __FUNCTION__  << " " << e.what() << endl;
    }
  }
  delete[] pBuf1;
  delete[] pBuf2;
//  delete[] pResult;
  _bReaderRunning = false;
}

void CLMS100Source::SetFogFilterMode(bool bOn) {

  ostringstream oss;
  oss << "sWN MSsuppmode " << bOn;
  SendLMSCommand(oss.str());
  _bFogFilterState = bOn;
}

void CLMS100Source::SetNto1FilterMode(bool bOn) {

  ostringstream oss;
  oss << "sWN LFPnto1filter " << bOn;
  SendLMSCommand(oss.str());
  _bNto1FilterMode = bOn;
}

void CLMS100Source::SetMeanFilterMode(bool bOn, int nFrameNum) {

  if (bOn && ((nFrameNum < 2) || (nFrameNum > 100))) {
    ESStreamException ess;
    ess << __FUNCTION__ << " nFrameNum OutBound: " << nFrameNum;
    throw ess;
  }

  ostringstream oss; oss << setbase(16);
  oss << "sWN LFPmeanfilter " << bOn << " " << nFrameNum;
  SendLMSCommand(oss.str());
  _bMeanFilterMode = bOn;
}

//void CLMS100Source::SetFreqAndResolution(int nHz, double dReso) {
void CLMS100Source::SetFreqAndResolution(EFreqAndAngleMode eMode) {

  int nHz;
  double dReso;
  boost::tie(nHz, dReso) = GetFreqAndResoFromMode(eMode);

  unsigned int nReso = (int)round(dReso*10000);
  if ((nReso != 2500) && (nReso != 5000)) {
    ESStreamException ess;
    ess << __FUNCTION__ << " Resolution OutBound: " << dReso;
    throw ess;
  }
  if ((nHz != 25) && (nHz != 50)) {
    ESStreamException ess;
    ess << __FUNCTION__ << " Freq OutBound: " << nHz;
    throw ess;
  }
  if ((nReso == 2500) && (nHz == 50)) { //0.25deg/50hzはできない
    ESStreamException ess;
    ess << __FUNCTION__ << " Resolution and Freq. Combination  OutBound: " << dReso << "/" << nHz;
    throw ess;
  }

  ostringstream oss; oss << setbase(16) << uppercase;
  oss << "sMN mLMPsetscancfg " << nHz*100 << " 1 " << nReso << " " << -450000  << " " << 2250000;
  SendLMSCommand(oss.str());
  SendLMSCommand("sMN mEEwriteall");
  cout << "msg: " << oss.str() << endl;

  double d1 = (_dEndDeg-_LRFProperty._dFirstAngle).get();
  _LRFProperty._nElemNum = (int)round(d1/_LRFProperty._dReso.get()) + 1;

  cout << "ElemNum=" << _LRFProperty._nElemNum << endl;
  InitArray();
  SetPosLimit(0, -1);
}


void CLMS100Source::SetScanDataAngle(double dStartAngle, double dEndAngle) {

  unsigned int nReso = (int)round(_LRFProperty._dReso.get_deg()*10000);
  if ((nReso != 2500) && (nReso != 5000)) {
    ESStreamException ess;
    ess << __FUNCTION__ << " Resolution OutBound: " << _LRFProperty._dReso.get_deg();
    throw ess;
  }

  int nStartAngle = (int)round((dStartAngle+90.0)*10000);
  int nEndAngle   = (int)round((dEndAngle+90.0)*10000);
  
  if ( (nStartAngle < -450000) || (nStartAngle > 2250000) ){
    ESStreamException ess;
    ess << __FUNCTION__ << " StartAngle OutBound: " << nStartAngle;
    throw ess;
  }
  if ( (nEndAngle < -450000) || (nEndAngle > 2250000) ){
    ESStreamException ess;
    ess << __FUNCTION__ << " EndAngle OutBound: " << nEndAngle;
    throw ess;
  }

  unsigned int n1; memcpy(&n1, &nStartAngle, 4);
  unsigned int n2; memcpy(&n2, &nEndAngle, 4);

  ostringstream oss; oss << setbase(16) << uppercase;
  oss << "sWN LMPoutputRange 1 " << nReso << " " << n1 << " " << n2;
  SendLMSCommand(oss.str());
  _LRFProperty._dFirstAngle.set_deg(dStartAngle);
  _LRFProperty._nElemNum = (nEndAngle-nStartAngle)/nReso + 1;
  //これを呼ばないとしばらく変更されないままのデータがやってくる
  SendLMSCommand("sMN mEEwriteall");
  InitArray();
  SetPosLimit(0, -1);
}