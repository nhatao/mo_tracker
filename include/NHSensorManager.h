#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <iomanip>
#include "mmtimer.h"
#include <boost/function.hpp>

class INHSensor;
class INHSensorStatus {
public:
  INHSensorStatus(const INHSensor *pParent) {
    _dMeasuredTime = 0;
    _pParent = pParent;
  }
  INHSensorStatus(double dTime) { //default Status;
    _dMeasuredTime = dTime;
    _pParent = NULL;
  }
  virtual ~INHSensorStatus() {
  }
  void WriteToLog(std::ostream &out, int nFrameNum) {
    out << std::setprecision(16);
    WriteToLogImpl(out, nFrameNum);
    out << " " << _dMeasuredTime << std::endl;
  }
  void ReadFromLog(std::istream &in) {
    ReadFromLogImpl(in);
    in >> _dMeasuredTime;
  } 
  void ReadFromLog(std::istream &in, double dTime) {
    _dMeasuredTime = dTime;
    ReadFromLogImpl(in);
  } 
  double GetMeasuredTime() const {return _dMeasuredTime;}
  const INHSensor * GetParent() const {return _pParent;}
  friend class INHSensor;

protected:
  virtual void WriteToLogImpl(std::ostream &out, int nFrameNum) = 0;
  virtual void ReadFromLogImpl(std::istream &in) = 0;
  const INHSensor *_pParent;
//private:
  double _dMeasuredTime;
};

//NHSensorManagerから呼び出すためのインタフェース
//実際のセンサは，このクラスを継承して作成するかAdapterパターンを用いる
class INHSensor {
public: 
  INHSensor(const std::string &rsName){
    _sSensorName = rsName;
    std::cout << "Sensor: " << this << " " << _sSensorName << " constructer called." << std::endl;
  }
  virtual ~INHSensor() {
    std::cout << "Sensor: " << this << " " << _sSensorName << " destructer called." << std::endl;
  }
  virtual void Init(){}; //計測直前に何かする場合, dCurrentTimeは時刻あわせなどに用いる
  virtual bool Measure() = 0; //計測を指示
  virtual void Finish(){}; //計測終了
  virtual unsigned int GetMinUpdateTime() const = 0; //この戻り値の時間(ミリ秒)で計測可能
  virtual boost::shared_ptr<INHSensorStatus> MakeSensorStatus() const = 0;
  virtual void SetWatch(double dFirstTime) {}; //時刻合わせ,dFirstTimeはQueryPerformanceCounterから取得した値(必要なら実装)
  bool UpdateStatus(INHSensorStatus *pStatus, double dTime) {
    pStatus->_dMeasuredTime = dTime;
    return UpdateStatusImpl(pStatus); //Sensor内部の時計を使う場合，UpdateStatusImplの中でpStatus->_dMeasuredTimeを更新
  };
  const std::string& GetSensorName() const {return _sSensorName;}
  //CurrentDir -> ログを書き出すディレクトリ。読み込むディレクトリではない
  void SetCurrentDir(const std::string &rsLogDir) {_sCurrentDir = rsLogDir;}
  const std::string& GetCurrentDir() const {return _sCurrentDir;}


protected:
  virtual bool UpdateStatusImpl(INHSensorStatus *pStatus) const = 0;
  std::string _sSensorName;
  std::string _sCurrentDir;
};

class CNHSensorFromFileImpl;
class CNHSensorManager
{
public:

  typedef boost::function<void (boost::shared_ptr<const INHSensorStatus>)> CallbackFunc;

  CNHSensorManager(void);
  virtual ~CNHSensorManager(void);

  void RunManager(const std::string &rsDirName = "");
  void StopManager();
  bool IsRunning() const;
  //戻り値:センサID
  unsigned int AddSensor(boost::shared_ptr<INHSensor> pSensor, unsigned int nStatusBufferNum, bool bLog, unsigned int nUpdateTime = 0);
  unsigned int AddSensorWithCallback(boost::shared_ptr<INHSensor> pSensor, unsigned int nStatusBufferNum, bool bLog, CallbackFunc f, unsigned int nUpdateTime = 0);

  boost::shared_ptr<INHSensor> GetSensor(unsigned int nID);
  boost::shared_ptr<const INHSensor> GetSensor(unsigned int nID) const;
  unsigned int GetUpdateTime(unsigned int nID) const;
  void SetUpdateTime(unsigned int nID, unsigned int nUpdateTime);

  const INHSensorStatus * GetLatestStatus(unsigned int nID);
  const INHSensorStatus * GetNearestStatus(unsigned int nID, double dTime);
  void GetAllLatestStatus(unsigned int nPriorityID, std::vector<const INHSensorStatus *>& rvStates);
  //戻り値の順序は, nPriorityID->vOtherID[0]->vOtherID[1]->....
  void GetLatestStatusGroup(unsigned int nPriorityID, const std::vector<unsigned int> &vOtherID, std::vector<const INHSensorStatus *>& rvStates);
  void GetLatestStatusGroup(const std::vector<unsigned int> &rvIDs, std::vector<const INHSensorStatus *>& rvStates);


  //一定の時間の範囲内のデータを返す．nPriorityIDのセンサデータは時間内の全てが返ってくる．残りはnPriorityIDの各センサデータにもっとも近い時刻のデータを返す．
  //dPeriodEnd<0のとき，(最新のデータ-dPeriodBegin)から最新のデータまでを取得
  //rvStatusLogに結果が格納される
  void GetDataforGivenPeriod(unsigned int nPriorityID, std::vector<unsigned int> vOtherID, std::vector<std::vector<const INHSensorStatus*> > &rvStatusLog, double dPeriodBegin, double dPeriodEnd=-1);
  //vOtherIDのデータはdvTimeShift秒前となる．vOtherIDとdvTimeShiftは同じ長さである必要がある．
  void GetDataforGivenPeriod(unsigned int nPriorityID, std::vector<unsigned int> vOtherID, std::vector<double> dvTimeShift, std::vector<std::vector<const INHSensorStatus*> > &rvStatusLog, double dPeriodBegin, double dPeriodEnd=-1);

  //rvMeasuredTimeの中身は，対応するINHSensorStatus->GetMeasuredTime() の値と同じ 何でこのインターフェースにしたんだっけ？？ とりあえず過去の互換性のために取っておく
  void GetAllLatestStatus(unsigned int nPriorityID, std::vector<const INHSensorStatus *>& rvStates, std::vector<double> &rvMeasuredTime);
  void GetLatestStatus(unsigned int nID, const INHSensorStatus *& pStatus, double &rdMeasuredTime);

  bool IsFileMode() const {return _bFileMode;}
  void SetToFileMode(const boost::posix_time::ptime &rFirstTime) {
    _bFileMode = true;
    _FirstBoostTime = rFirstTime;
  }
  void SetToRealTimeMode() {
    _bFileMode = false;
  }
  bool GetFileMode() const {return _bFileMode;}
  void SetFirstTime(double dFirstTime) { //FileModeのみ有効
    _dFirstTime = dFirstTime;
  }
  class CSensorStatusUpdater {
  public:
    CSensorStatusUpdater (boost::shared_ptr<INHSensor> pSensor, unsigned int nStatusBufferNum, bool bLog, unsigned int nUpdateTime);
    virtual ~CSensorStatusUpdater() {
    }
    boost::shared_ptr<INHSensor> GetSensor() {return _pSensor;}
    boost::shared_ptr<INHSensor const> GetSensor() const {return _pSensor;}
    unsigned int GetUpdateTime() const {return _nUpdateTime;}
    void SetUpdateTime(unsigned int n) {_nUpdateTime = n;}

    virtual void Run(int64_t nFirstTime, const std::string &rsLogDirName);    
    virtual void FinishThread() {_bFinishedFlag = true;}
    virtual const INHSensorStatus* GetLatestStatus(double *pdMeasuredTime=NULL);
    virtual const INHSensorStatus* GetNearestTimeStatus(double dBaseTime, double *pdMeasuredTime=NULL);
    virtual void GetDataforGivenPeriod(std::vector<const INHSensorStatus*> &rvStatusLog, double dPeriodBegin, double dPeriodEnd);
    bool Log() const {return _bLog;}

    void SetFunc(CNHSensorManager::CallbackFunc f) {
      _Func1 = f;
    }
  protected:

    boost::shared_ptr<INHSensor> _pSensor;
    unsigned int _nUpdateTime;
    std::vector<boost::shared_ptr<INHSensorStatus> > _vSensorStatus;
    std::vector<double> _vdMeasuredTime;
    volatile size_t _nLast;
    volatile bool _bFinishedFlag;
    bool _bLog;
    virtual size_t GetNearestID(double dTime);

    CNHSensorManager::CallbackFunc _Func1;
  };

  class CSensorStatusUpdaterFile : public CSensorStatusUpdater{
  public:
    CSensorStatusUpdaterFile (boost::shared_ptr<CNHSensorFromFileImpl> pSensor, unsigned int nStatusBufferNum,  unsigned int nUpdateTime);
    virtual ~CSensorStatusUpdaterFile() {}

    virtual void Run(int64_t nFirstTime, const std::string &rsLogDirName);    
    virtual void FinishThread();
    virtual const INHSensorStatus* GetLatestStatus(double *pdMeasuredTime=NULL);
    virtual const INHSensorStatus* GetNearestTimeStatus(double dBaseTime, double *pdMeasuredTime=NULL);
    virtual void GetDataforGivenPeriod(std::vector<const INHSensorStatus*> &rvStatusLog, double dPeriodBegin, double dPeriodEnd);
    void SetCurrentTime(double dTime);

  protected:
    volatile size_t _nFileLineNum;
    boost::shared_ptr<CNHSensorFromFileImpl> _pSensorFromFile;
    boost::mutex _Mutex;
    boost::condition _UpdateFinishCondition;
  };

  //RunManagerを呼び出したタイミングで更新
  const std::string& GetCurrentLogDirName() const {return _sCurrentLogDirName;}
  double GetCurrentTime();
  size_t GetSensorNum() const {return _vSensors.size();}

  boost::posix_time::ptime SensorTimeToBoostTime(double dTime) const;

protected:

  void SetFileModeFirstTime();

  std::vector<boost::shared_ptr<CSensorStatusUpdater> > _vSensors;
  std::vector<boost::shared_ptr<boost::thread> > _vThreads;
  std::vector<boost::shared_ptr<INHSensor> > _vSensorsRaw; //先に_vSensorsをデストラクトしたい

  volatile bool _bRunning;
  bool _bFileMode;
  double _dFirstTime;
  boost::posix_time::ptime _FirstBoostTime;
  unsigned int _uPeriod; //timeSetEventに渡す引数

  void OpenLogDir();
  std::string _sCurrentLogDirName;
  int64_t _nFirstTime;

  boost::recursive_mutex _GetDataMutex; //複数スレッドが同時にデータを要求するときの対策

};
