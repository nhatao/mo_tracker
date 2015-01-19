#pragma once
#include "LRFSource.h"
#include "Socket.h"
#include <boost/utility.hpp>
#include <boost/thread.hpp>
#include <queue>

//TODO LMDscandatacfg

struct SLMS100ScanData {

  SLMS100ScanData() {
    Init();
  }
  void Init() {
    LaserData1.bEnable = false;
    LaserData2.bEnable = false;
    Remission1.bEnable = false;
    Remission2.bEnable = false;
  }

  //8byteのデータはshortにすることにした．charだと文字列だと解釈されてややこしいため．
  unsigned short nVersion;
  unsigned short nDeviceID;
  unsigned int nSerialNo;
  unsigned short nDeviceStatus1;//dummy
  unsigned short nDeviceStatus2; 
  unsigned short nMsgCounter;
  unsigned short nScanCounter;
  unsigned int nPowerUpDuration;
  unsigned int nTransmissionDuration;
  unsigned short nInputStatus1;//dummy
  unsigned short nInputStatus2;
  unsigned short nOutputStatus1;//dummy
  unsigned short nOutputStatus2;
  unsigned short nReserved;
  unsigned int nScanningFrequency, nMeasurementFrequency;

  //Encoder
  std::vector< std::pair<unsigned short, unsigned short> > vEncoderData; //first:nEncoderPosition second:nEncoderSpeed;

  //Laser
  struct SLaserData {
    bool bEnable;
    float dScaleFactor;
    float dScaleFactorOffset;
    int nStartAngle;
    float dStartAngle;
    unsigned short nSteps;
    std::vector<unsigned short> vData;

    void ReadFromMsg(std::istream &is);
  };
  SLaserData LaserData1;
  SLaserData LaserData2;
  SLaserData Remission1;
  SLaserData Remission2;

  //Position
  bool bHasPosition;
  float dX; float dY; float dZ;
  float dXR; float dYR; float dZR;
  unsigned short nKindofRotation;
  

  //Name
  bool bHasDeviceName;
  std::string sDeviceName; //ほんとは数字？

  //Comment
  bool bHasComment;
  std::string sComment;

  //Time
  bool bHasTime;
  unsigned short nYear;
  unsigned short nMonth;
  unsigned short nDay;
  unsigned short nHour;
  unsigned short nMinute;
  unsigned short nSecond;
  unsigned int nMicroSecond;

  //Event
  bool bHasEvent;
  std::string sEventType;
  unsigned int nEncoderPositionEvent;
  unsigned int nTimeofEvent;
  int nAngleofEvent;

  void ReadFromMsg(const char* pMsg);

};

class CLMS100Source :
  public CLRFSource, private boost::noncopyable
{
public:

  enum EFreqAndAngleMode {
    e025DegAt25Hz,
    e050DegAt50Hz,
    e050DegAt25Hz
  };

  CLMS100Source(bool bShowDebugMsg = false);
  virtual ~CLMS100Source(void);

  void Init(const std::string &rsIP, int nPort, EFreqAndAngleMode eMode = e025DegAt25Hz, double dStartAngle = -135, double dEndAngle = 135);

  virtual void Measure();
  virtual void RequestData(); //呼び出しはRequestData->Measureの順．RequestDataは可能な限り一瞬で終わるコマンド，Measureは時間がかかってもよい
  virtual void Reset();
  virtual bool CheckInvalidData(unsigned short nData) const; //センサ固有のエラー値の場合falseを返す

  void SetFogFilterMode(bool bOn);
  bool GetFogFilterMode() const {return _bFogFilterState; }
  void SetNto1FilterMode(bool bOn); //trueにすると後ろからのパルスを優先する
  bool GetNto1FilterMode() const {return _bNto1FilterMode; }
  void SetMeanFilterMode(bool bOn, int nFrameNum=2);
  bool GetMeanFilterMode() const {return _bMeanFilterMode;}

  //dResoは0.50か0.25のみ
  void SetFreqAndResolution(EFreqAndAngleMode eMode);
  void SetScanDataAngle(double dStartAngle = -135, double dEndAngle = 135);

  const std::string &GetIP() const {return _sIP;}
  int GetPort() const {return _nPort;}

protected:

  std::string SendLMSCommand(const std::string &rsMsg, double dTimeOut=10.0);

  void InitStatus(EFreqAndAngleMode eMode, double dStartAngle, double dEndAngle); //LMSからデータを取得
  void StartMeasure(); //計測開始

  boost::shared_ptr<CSocketClient> _pSocket;

  bool _bInitialized;
  void ReadDataThread(); //_pReaderの中身
  boost::shared_ptr<boost::thread> _pReader;
  volatile bool _bReaderRunning;
  boost::recursive_mutex _SocketMutex;

  char* _pReadBuf;
  size_t _nReadBufSize;

  //first データバイト数 second 
  std::pair<int, int> ReadOneMsg(char *pBuf, int nBufSize);

  void InterpretMsg(const char* pMsg);

  void InterpretScanData(const char* pMsg);
  void InterpretResponseData(const char* pMsg);

  bool _bFogFilterState;
  bool _bNto1FilterMode;
  bool _bMeanFilterMode;

  boost::recursive_mutex _ReplyMutex;
  boost::condition_variable_any _ReplyWaitSet;
//  std::map<std::string, std::string> _mReply; //一個しかないはず？
  std::string _sReply;

  int _nUpdateTime;
  //HzとReso(deg)を返す 
  std::pair<int, double> GetFreqAndResoFromMode(EFreqAndAngleMode eMode);

  CAngle _dEndDeg;

  boost::recursive_mutex _HistoryMutex;
  std::queue<boost::shared_ptr<SLMS100ScanData> > _qLMSHistory;
  boost::condition_variable_any _HistoryWaitSet;
  size_t _nQueueSizeMax;

  bool _bShowDebugMsg;
  unsigned short _nLastScanCounter;

  bool NeedToWait();

  std::string _sIP;
  int _nPort;
};
