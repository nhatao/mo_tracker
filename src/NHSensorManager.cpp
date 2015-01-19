#include "StdAfx_MOTracking.h"
#include "NHSensorManager.h"
#include "NHSensorFromFile.h"
//#include "Logger.h"
#include "mt_ofstream.h"
#include "mmtimer.h"
#include "OutputDebugStream.h"

#ifdef _MSC_VER
#include <mmsystem.h>
#else 
#include <sys/time.h> 
#endif

void DoNothingSensorStatus(boost::shared_ptr<const INHSensorStatus> p) {}

static int64_t g_nFreq;
static boost::mutex g_TimeMutex;

static void GetCurrentTimeRaw(int64_t &rResult) {

  boost::mutex::scoped_lock lk(g_TimeMutex);
#ifdef _MSC_VER
  QueryPerformanceCounter((LARGE_INTEGER *)&rResult);
#else
  timeval tv2;
  gettimeofday(&tv2, NULL);
  int64_t n = tv2.tv_sec;
  n*=(1000*1000);
  rResult = n+tv2.tv_usec;
#endif
}

using namespace std;

CNHSensorManager::CSensorStatusUpdater::CSensorStatusUpdater (boost::shared_ptr<INHSensor> pSensor, unsigned int nStatusBufferNum, bool bLog, unsigned int nUpdateTime) {
  _pSensor = pSensor;
  _bLog = bLog;
  if (nUpdateTime==0) _nUpdateTime = pSensor->GetMinUpdateTime();
  else _nUpdateTime = nUpdateTime;
  for (size_t i=0; i<nStatusBufferNum; ++i) {
    _vSensorStatus.push_back(pSensor->MakeSensorStatus());
  }
  _vdMeasuredTime.resize(nStatusBufferNum);
  _nLast = nStatusBufferNum-1;
  _Func1 = &DoNothingSensorStatus;
}

//nFirstTimeは最初のタイミングを合わせるためのもの
void CNHSensorManager::CSensorStatusUpdater::Run(int64_t nFirstTime, const std::string &rsLogDirName) {
  _bFinishedFlag = false;

  try {
#ifdef _MSC_VER
    HANDLE hThread = GetCurrentThread();
    if (SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL)) {
//      cout << "Priority Set OK" << endl;
    }
    else {
      cout << "Sensor: " << GetSensor()->GetSensorName() << " ";
      cout << "Priority Set NG" << endl;
    }
#endif

    mmtimer mt;
    double dSensingAveTime = 0;
    double dSensingMaxTime = -DBL_MAX;
    double dSleepMax = -DBL_MAX;
    int nLostFrame = 0;
    int nCurrentFrame = 0;
    double dFrameTime = _nUpdateTime/1000.0;
    int64_t nCurrentTime,nFinishedTime;
    double dFinishedTime = 0;
    ostringstream osLogFileName;
    osLogFileName << rsLogDirName << "/" << _pSensor->GetSensorName() << ".dat";
    boost::shared_ptr<mt_ofstream> pLogFile;
    if (_bLog) pLogFile = boost::shared_ptr<mt_ofstream>(new mt_ofstream(osLogFileName.str().c_str()));

    _pSensor->Init();
    double dFirstTime = (double)nFirstTime/g_nFreq;
    GetCurrentTimeRaw(nCurrentTime);
    double dFirstTime2 = (double)nCurrentTime/g_nFreq - dFirstTime; 
    dFinishedTime = dFirstTime2;
    int nFirstLost = (int)((dFirstTime2*1000)/_nUpdateTime); //Initの時間でロスするフレーム数

    while (!_bFinishedFlag) {
      GetCurrentTimeRaw(nCurrentTime);
      double dCurrentMeasureTime = ((double)(nCurrentTime))/g_nFreq-dFirstTime;
      if (_pSensor->Measure()) {
        mmtimer mt;
        unsigned int nLogTemp = (unsigned int)_nLast+1;
        if (nLogTemp == _vSensorStatus.size()) {
          nLogTemp = 0;
        }
        _pSensor->UpdateStatus(_vSensorStatus[nLogTemp].get(),dCurrentMeasureTime);
        _vdMeasuredTime[nLogTemp] = _vSensorStatus[nLogTemp]->GetMeasuredTime();
        //ここで変更
        _nLast = nLogTemp;
        _Func1(_vSensorStatus[nLogTemp]);
        if (_bLog) {
          _vSensorStatus[nLogTemp]->WriteToLog(*pLogFile, nCurrentFrame);
        }
      }
      else {
//        cout << "Sensor Update Failed!" << endl;
      }
      dSleepMax=max(dSleepMax, dCurrentMeasureTime-dFinishedTime);
//      QueryPerformanceCounter((LARGE_INTEGER *)&nFinishedTime); 
      GetCurrentTimeRaw(nFinishedTime);
      dFinishedTime = ((double)(nFinishedTime))/g_nFreq-dFirstTime;
      ++nCurrentFrame;
      dSensingAveTime += dFinishedTime-dCurrentMeasureTime;
      dSensingMaxTime = max(dSensingMaxTime, dFinishedTime-dCurrentMeasureTime);
      double dSleepTime = (nCurrentFrame+nLostFrame)*dFrameTime - dFinishedTime;
      while (dSleepTime < 0) {
        dSleepTime+=dFrameTime;
        ++nLostFrame;
      }
      if (dSleepTime < 0) Sleep(0);
      else Sleep((int)floor(dSleepTime*1000));
    }
    _pSensor->Finish();
    cout << _pSensor->GetSensorName() << " Frame: " << nCurrentFrame <<  " Lost: " << nLostFrame-nFirstLost << " FPS: " << nCurrentFrame/(dFinishedTime-dFirstTime2) << " Ave: " << dSensingAveTime/(nCurrentFrame)*1000 << " Max: " << dSensingMaxTime*1000 << " MaxSleep: " << dSleepMax*1000;// << endl;
    cout << " FirstTime2 : " << dFirstTime2 << endl;
  }
  catch (std::exception &e) {
    _bFinishedFlag = true;
    cout << __FUNCTION__  << " exception:" << e.what() << endl;
    cout << "Sensor Update Aborted!" << endl;
  }
}

//Latestの1個前を返す -> FPSが同じぐらいのセンサが複数あるときの対策 TODO 遅いセンサならこうしなくてもいい
const INHSensorStatus* CNHSensorManager::CSensorStatusUpdater::GetLatestStatus(double *pdMeasuredTime) {
  int n = (int)_nLast-1;
  if (n<0) n = (int)_vdMeasuredTime.size()-1;
  if ((n<0) || (n>=(int)_vdMeasuredTime.size()) || (n>=(int)_vSensorStatus.size())) {
    ESStreamException ess;
    ess << "CSensorStatusUpdater Error: n=" << n << " " << _vdMeasuredTime.size() << " " << _vSensorStatus.size();
    throw ess;
  }
  if (pdMeasuredTime) {
    *pdMeasuredTime = _vdMeasuredTime[n];
  }
  return _vSensorStatus[n].get();
}

const INHSensorStatus* CNHSensorManager::CSensorStatusUpdater::GetNearestTimeStatus(double dBaseTime, double *pdMeasuredTime) {
  size_t nResult;
  double dLastTime = _vdMeasuredTime[_nLast];
  if (dLastTime < dBaseTime) {
//    cout << "unko" << endl;
    nResult = (int)_nLast;
  }
  else {
    nResult = GetNearestID(dBaseTime);
  }
  if (pdMeasuredTime) {
    *pdMeasuredTime = _vdMeasuredTime[nResult];
  }
  return _vSensorStatus[nResult].get();
}

void CNHSensorManager::CSensorStatusUpdater::GetDataforGivenPeriod(std::vector<const INHSensorStatus*> &rvStatusLog, double dPeriodBegin, double dPeriodEnd) {
  size_t nBegin, nEnd;
  if (dPeriodEnd < 0) {
    if (_nLast==0) nEnd = _vdMeasuredTime.size()-1;
    else nEnd = _nLast-1;
    nBegin = GetNearestID(_vdMeasuredTime[nEnd]-dPeriodBegin);
  }
  else {
    nEnd = GetNearestID(dPeriodEnd);
    nBegin = GetNearestID(dPeriodBegin);
  }
  if (nBegin < nEnd) {
    for (size_t i=nBegin; i<nEnd; ++i) {
      rvStatusLog.push_back(_vSensorStatus[i].get());
    }
  }
  else {
    for (size_t i=nBegin; i<_vSensorStatus.size(); ++i) {
      rvStatusLog.push_back(_vSensorStatus[i].get());
    }
    for (size_t i=0; i<nEnd; ++i) {
      rvStatusLog.push_back(_vSensorStatus[i].get());
    }
  }
}


size_t CNHSensorManager::CSensorStatusUpdater::GetNearestID(double dBaseTime) {

  double dLastTime = _vdMeasuredTime[_nLast];
  int nResult = 0;
  size_t nStatusBufferNum = _vSensorStatus.size();
  double dBeforeTime = dLastTime;
  double dCurTime = dBeforeTime;
  for (int i=0; i<(int)nStatusBufferNum; ++i) {
    nResult = (int)_nLast-i;
    if (nResult<0) nResult+=(int)nStatusBufferNum;
    dCurTime = _vdMeasuredTime[nResult];
    if (dCurTime < dBaseTime) break;
    dBeforeTime = dCurTime;
  }
  if (abs(dCurTime-dBaseTime) > abs(dBeforeTime-dBaseTime)) {
    ++nResult;
    if (nResult>=(int)nStatusBufferNum) nResult=(int)nStatusBufferNum-1;
  }
  return (size_t)nResult;
}


CNHSensorManager::CSensorStatusUpdaterFile::CSensorStatusUpdaterFile (boost::shared_ptr<CNHSensorFromFileImpl> pSensor, unsigned int nStatusBufferNum,  unsigned int nUpdateTime)
  : CSensorStatusUpdater(pSensor,nStatusBufferNum,false, nUpdateTime) 
{
  _nLast = 0;
  _nFileLineNum = 0;
  _pSensorFromFile = pSensor;
}

void CNHSensorManager::CSensorStatusUpdaterFile::Run(int64_t nFirstTime, const std::string &rsLogDirName) {
  _bFinishedFlag = false;

  try {
    _nLast = 0;
//    _nFileLineNum = 0;
    _pSensor->Init();
    boost::mutex::scoped_lock lk(_Mutex);
    _UpdateFinishCondition.wait(lk);
    _pSensor->Finish();
  }
  catch (std::exception &e) {
    _bFinishedFlag = true;
    cout << __FUNCTION__  << " exception:" << e.what();
  }
}

const INHSensorStatus* CNHSensorManager::CSensorStatusUpdaterFile::GetLatestStatus(double *pdMeasuredTime) {

  ++_nLast;
  if (_nLast >= _vSensorStatus.size()) _nLast = 0;
  _pSensorFromFile->MeasureNthStatus(*_vSensorStatus[_nLast].get(), _nFileLineNum);
  ++_nFileLineNum;
  if (pdMeasuredTime) {
    *pdMeasuredTime = _vSensorStatus[_nLast]->GetMeasuredTime();
  }
  _Func1(_vSensorStatus[_nLast]);
  return _vSensorStatus[_nLast].get();
}

const INHSensorStatus* CNHSensorManager::CSensorStatusUpdaterFile::GetNearestTimeStatus(double dBaseTime, double *pdMeasuredTime) {

  ++_nLast;
  if (_nLast >= _vSensorStatus.size()) _nLast = 0;
  _pSensorFromFile->MeasureNearestTime(*_vSensorStatus[_nLast].get(), dBaseTime);
  if (pdMeasuredTime) {
    *pdMeasuredTime = _vSensorStatus[_nLast]->GetMeasuredTime();
  }
  return _vSensorStatus[_nLast].get();
}

void CNHSensorManager::CSensorStatusUpdaterFile::GetDataforGivenPeriod(std::vector<const INHSensorStatus*> &rvStatusLog, double dPeriodBegin, double dPeriodEnd) {

  size_t nBegin, nEnd;
  if (dPeriodEnd < 0) {
    nEnd = _nFileLineNum;
    ++_nFileLineNum;
    _pSensorFromFile->MeasureNthStatus(*_vSensorStatus[_nLast].get(), nEnd);
    double d1 = _vSensorStatus[_nLast]->GetMeasuredTime();
    nBegin = _pSensorFromFile->GetNearestLine(d1-dPeriodBegin);
  }
  else {
    nEnd = _pSensorFromFile->GetNearestLine(dPeriodEnd);
    nBegin = _pSensorFromFile->GetNearestLine(dPeriodBegin);
  }
  for (size_t n=nBegin; n<nEnd; ++n) {
    ++_nLast;
    if (_nLast >= _vSensorStatus.size()) _nLast = 0;
    _pSensorFromFile->MeasureNthStatus(*_vSensorStatus[_nLast].get(), n);
    rvStatusLog.push_back(_vSensorStatus[_nLast].get());
  }
}

void CNHSensorManager::CSensorStatusUpdaterFile::FinishThread() {
  boost::mutex::scoped_lock lk(_Mutex);
  _UpdateFinishCondition.notify_all();
  _bFinishedFlag = true;
}

CNHSensorManager::CNHSensorManager(void)
{
#ifdef _MSC_VER
  QueryPerformanceFrequency((LARGE_INTEGER *)&g_nFreq);
#else
  g_nFreq = 1000*1000;
#endif

  _bRunning = false;
#ifdef _MSC_VER
  TIMECAPS timercaps;
  ZeroMemory(&timercaps, sizeof(timercaps));
  MMRESULT mmresult = timeGetDevCaps(&timercaps, sizeof(TIMECAPS));
  _uPeriod = timercaps.wPeriodMin;
  mmresult = timeBeginPeriod(_uPeriod);
#endif
  _bFileMode = false;
  _dFirstTime = 0;
}

CNHSensorManager::~CNHSensorManager(void)
{

  StopManager();
  _vSensors.clear();
  _vSensorsRaw.clear();
#ifdef _MSC_VER
  timeEndPeriod(_uPeriod);
#endif
}

void CNHSensorManager::StopManager() {

  if (!_bRunning) {
    return;
  }
  _bRunning = false;
  for (size_t i=0; i<_vThreads.size(); ++i) {
    _vSensors[i]->FinishThread();
    Sleep(100);
    _vThreads[i]->join();;
  }
  _vThreads.clear();
}

bool CNHSensorManager::IsRunning() const {
  return _bRunning;
}

#include <boost/version.hpp>

unsigned int CNHSensorManager::AddSensor(boost::shared_ptr<INHSensor> pSensor, unsigned int nStatusBufferNum, bool bLog, unsigned int nUpdateTime) {

  if (_bFileMode) {
    boost::shared_ptr<CNHSensorFromFileImpl> pSensorFile = 
      boost::dynamic_pointer_cast<CNHSensorFromFileImpl>(pSensor);
//      boost::shared_dynamic_cast<CNHSensorFromFileImpl>(pSensor);
    if (!pSensorFile) {
      ESStreamException ess; ess << "File Mode cast Failed!";
      throw ess;
    }
    _vSensors.push_back(boost::shared_ptr<CSensorStatusUpdater> (
      new CSensorStatusUpdaterFile(pSensorFile, nStatusBufferNum, nUpdateTime)));
    _vSensorsRaw.push_back(pSensor);
  }
  else {
    if (nUpdateTime == 0) nUpdateTime = pSensor->GetMinUpdateTime();

    _vSensors.push_back(boost::shared_ptr<CSensorStatusUpdater> (new CSensorStatusUpdater(pSensor, nStatusBufferNum, bLog, nUpdateTime)));
    _vSensorsRaw.push_back(pSensor);
  }
  return (unsigned int)_vSensors.size()-1;
}


unsigned int CNHSensorManager::AddSensorWithCallback(boost::shared_ptr<INHSensor> pSensor, unsigned int nStatusBufferNum, bool bLog,  CallbackFunc f, unsigned int nUpdateTime) {

  unsigned int n = AddSensor(pSensor, nStatusBufferNum, bLog, nUpdateTime);
  _vSensors.back()->SetFunc(f);
  return n;
}


boost::shared_ptr<INHSensor> CNHSensorManager::GetSensor(unsigned int nID) {
  if (nID >= _vSensors.size()) {
    ESStreamException ess;
    ess << "CNHSensorManager::GetSensor nID too large: NID=" << nID << " size()=" << _vSensors.size();
    throw ess;
  }
  return _vSensors[nID]->GetSensor();
}

boost::shared_ptr<const INHSensor> CNHSensorManager::GetSensor(unsigned int nID) const{
  if (nID >= _vSensors.size()) {
    ESStreamException ess;
    ess << "CNHSensorManager::GetSensor nID too large: NID=" << nID << " size()=" << _vSensors.size();
    throw ess;
  }
  return _vSensors[nID]->GetSensor();
}


unsigned int CNHSensorManager::GetUpdateTime(unsigned int nID) const {
  if (nID >= _vSensors.size()) {
    ESStreamException ess;
    ess << "CNHSensorManager::GetUpdateTime nID too large: NID=" << nID << " size()=" << _vSensors.size();
    throw ess;
  }
  return _vSensors[nID]->GetUpdateTime();
}

void CNHSensorManager::SetUpdateTime(unsigned int nID, unsigned int nUpdateTime) {
  if (nID >= _vSensors.size()) {
    ESStreamException ess;
    ess << "CNHSensorManager::SetUpdateTime nID too large: NID=" << nID << " size()=" << _vSensors.size();
    throw ess;
  }
  _vSensors[nID]->SetUpdateTime(nUpdateTime);
}

void CNHSensorManager::GetLatestStatusGroup(const std::vector<unsigned int> &rvIDs, std::vector<const INHSensorStatus *>& rvStates) {

  if (rvIDs.empty()) {
    ESStreamException ess;
    ess << "CNHSensorManager::GetLatestStatusGroup ID Empty";
    throw ess;
  }

  boost::recursive_mutex::scoped_lock lk(_GetDataMutex);    
  rvStates.resize(rvIDs.size());
  double dBaseTime;
  const INHSensorStatus* pBaseStatus = _vSensors[rvIDs[0]]->GetLatestStatus(&dBaseTime);
  rvStates[0] = pBaseStatus;
  double d;
  for (size_t i=1; i<rvIDs.size(); ++i) {
    unsigned int nID = rvIDs[i];
    if (nID < _vSensors.size()) {
      rvStates[i] = _vSensors[nID]->GetNearestTimeStatus(dBaseTime, &d);
    }
    else {
      rvStates[i] = NULL;
    }
  }

}


void CNHSensorManager::GetLatestStatusGroup(unsigned int nPriorityID, const std::vector<unsigned int> &vOtherID, std::vector<const INHSensorStatus *>& rvStates) {

  std::vector<unsigned int> vIDs;
  vIDs.push_back(nPriorityID);
  for (size_t i=0; i<vOtherID.size(); ++i) {
    vIDs.push_back(vOtherID[i]);
  }
}

void CNHSensorManager::GetAllLatestStatus(unsigned int nPriorityID, vector<const INHSensorStatus *>& rvStates) {

  boost::recursive_mutex::scoped_lock lk(_GetDataMutex);    
  vector<double> vDummy;
  GetAllLatestStatus(nPriorityID, rvStates, vDummy);
}

void CNHSensorManager::GetAllLatestStatus(unsigned int nPriorityID, vector<const INHSensorStatus *>& rvStates, vector<double> &rvMeasuredTime) 
{

  boost::recursive_mutex::scoped_lock lk(_GetDataMutex);    
  rvStates.resize(_vSensors.size());
  rvMeasuredTime.resize(_vSensors.size());

  double dBaseTime;
  const INHSensorStatus* pBaseStatus = _vSensors[nPriorityID]->GetLatestStatus(&dBaseTime);
  rvStates[nPriorityID] = pBaseStatus;
  double dTrueBaseTime = pBaseStatus->GetMeasuredTime();
  rvMeasuredTime[nPriorityID] = dTrueBaseTime;
  for (size_t i=0; i<_vSensors.size(); ++i) {
    if (i!=nPriorityID) {
      rvStates[i] = _vSensors[(unsigned int)i]->GetNearestTimeStatus(dTrueBaseTime, &rvMeasuredTime[i]);
    }
  }
}

const INHSensorStatus* CNHSensorManager::GetNearestStatus(unsigned int nID, double dTime) {
  if (nID >= _vSensors.size()) {
    ESStreamException ess;
    ess << "CNHSensorManager::GetNearestStatus nID too large: NID=" << nID << " size()=" << _vSensors.size();
    throw ess;
  }
  boost::recursive_mutex::scoped_lock lk(_GetDataMutex);
  return _vSensors[nID]->GetNearestTimeStatus(dTime);
}

const INHSensorStatus * CNHSensorManager::GetLatestStatus(unsigned int nID)  {
  boost::recursive_mutex::scoped_lock lk(_GetDataMutex);
  return _vSensors[nID]->GetLatestStatus();
}

void CNHSensorManager::GetLatestStatus(unsigned int nID, const INHSensorStatus *& pStatus, double &rdMeasuredTime) {
  boost::recursive_mutex::scoped_lock lk(_GetDataMutex);
  pStatus = _vSensors[nID]->GetLatestStatus(&rdMeasuredTime);
}

void CNHSensorManager::GetDataforGivenPeriod(unsigned int nPriorityID, std::vector<unsigned int> vOtherID, std::vector<std::vector<const INHSensorStatus*> > &rvStatusLog, double dPeriodBegin, double dPeriodEnd) {
  boost::recursive_mutex::scoped_lock lk(_GetDataMutex);
  if (dPeriodBegin < 0) {
    ESStreamException ess;
    ess << __FUNCTION__ << " dPeriodBegin Minus: " << dPeriodBegin ;
    throw ess;
  }
  if ( (dPeriodEnd > 0) && (dPeriodEnd < dPeriodBegin) ) {
    ESStreamException ess;
    ess << __FUNCTION__ << " Time Skew: " << dPeriodBegin << "->" << dPeriodEnd;
    throw ess;
  }
  if (!rvStatusLog.empty()) rvStatusLog.clear();
  rvStatusLog.push_back(vector<const INHSensorStatus*>());
  _vSensors[nPriorityID]->GetDataforGivenPeriod(rvStatusLog.front(), dPeriodBegin, dPeriodEnd);
  for (size_t i=0; i<vOtherID.size(); ++i) {
    rvStatusLog.push_back(vector<const INHSensorStatus*>());
    vector<const INHSensorStatus*> &vLog = rvStatusLog.back();
    for (size_t j=0; j<rvStatusLog[0].size(); ++j) {
      vLog.push_back(_vSensors[vOtherID[i]]->GetNearestTimeStatus(rvStatusLog[0][j]->GetMeasuredTime()));
    }
  }
}

void CNHSensorManager::GetDataforGivenPeriod(unsigned int nPriorityID, std::vector<unsigned int> vOtherID, std::vector<double> dvTimeShift, std::vector<std::vector<const INHSensorStatus*> > &rvStatusLog, double dPeriodBegin, double dPeriodEnd) {

  boost::recursive_mutex::scoped_lock lk(_GetDataMutex);    
  if (dPeriodBegin < 0) {
    ESStreamException ess;
    ess << __FUNCTION__ << " dPeriodBegin Minus: " << dPeriodBegin ;
    throw ess;
  }
  if ( (dPeriodEnd > 0) && (dPeriodEnd < dPeriodBegin) ) {
    ESStreamException ess;
    ess << __FUNCTION__ << " Time Skew: " << dPeriodBegin << "->" << dPeriodEnd;
    throw ess;
  }
  if ( vOtherID.size() != dvTimeShift.size() ) {
    ESStreamException ess;
    ess << __FUNCTION__ << " vOtherID.size() != dvTimeShift.size() : " << vOtherID.size() << "!=" << dvTimeShift.size() ;
    throw ess;
  }
  if (!rvStatusLog.empty()) rvStatusLog.clear();
  rvStatusLog.push_back(vector<const INHSensorStatus*>());
  _vSensors[nPriorityID]->GetDataforGivenPeriod(rvStatusLog.front(), dPeriodBegin, dPeriodEnd);
  for (size_t i=0; i<vOtherID.size(); ++i) {
    rvStatusLog.push_back(vector<const INHSensorStatus*>());
    vector<const INHSensorStatus*> &vLog = rvStatusLog.back();
    for (size_t j=0; j<rvStatusLog[0].size(); ++j) {
      vLog.push_back(_vSensors[vOtherID[i]]->GetNearestTimeStatus(rvStatusLog[0][j]->GetMeasuredTime()-dvTimeShift[i]));
//      cout << rvStatusLog[0][j]->GetMeasuredTime() << " / " << vLog.back()->GetMeasuredTime() << endl;
    }
  }
}


#include <boost/bind.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

void CNHSensorManager::RunManager(const std::string &rsDirName) {

  StopManager();
  if (_bFileMode) {
    SetFileModeFirstTime();
    GetCurrentTimeRaw(_nFirstTime);
  }
  else {
    using namespace boost::posix_time;

    GetCurrentTimeRaw(_nFirstTime);
    _FirstBoostTime = microsec_clock::local_time();
    string sDirName = to_iso_string(_FirstBoostTime);

    if (rsDirName == "") _sCurrentLogDirName = sDirName;
    else _sCurrentLogDirName = rsDirName;
  }
  bool bMakeLogDir = false;
  for (size_t i=0; i<_vSensors.size(); ++i) {
    if (_vSensors[i]->Log()) bMakeLogDir = true;
    _vSensors[i]->GetSensor()->SetCurrentDir(_sCurrentLogDirName);
  }
  if (bMakeLogDir) {
    boost::filesystem::path logdir(_sCurrentLogDirName);
    if (!boost::filesystem::exists(logdir)) {
      boost::filesystem::create_directory(logdir);
    }
  }
  for (size_t i=0; i<_vSensors.size(); ++i) {
//    cout << "Starting Thread " << i << endl;
    _vThreads.push_back(boost::shared_ptr<boost::thread>(new boost::thread(
      boost::bind(&CSensorStatusUpdater::Run, boost::ref(_vSensors[i]), _nFirstTime, _sCurrentLogDirName))));
#ifdef _MSC_VER
    HANDLE th = _vThreads.back()->native_handle();
    BOOL res = SetThreadPriority(th, THREAD_PRIORITY_TIME_CRITICAL);
    if (res == FALSE) cout << "SetThreadPriority Failed : " << GetLastError() << endl;
#endif
  }
  _bRunning = true;
//  Sleep(500);
}

double CNHSensorManager::GetCurrentTime() {
  if (_bFileMode) {
    cout << "CNHSensorManager::GetCurrentTime() on FileMode: Not Active" << endl;
    return 0;
  }
  else {
    int64_t nCurrentTime = 0;
    GetCurrentTimeRaw(nCurrentTime);
    return ((double)(nCurrentTime-_nFirstTime))/g_nFreq;
  }
}

void CNHSensorManager::SetFileModeFirstTime() {

  for (size_t i=0; i<_vSensors.size(); ++i) {
    CSensorStatusUpdaterFile* pSensor = (CSensorStatusUpdaterFile* )(_vSensors[i].get());
    pSensor->SetCurrentTime(_dFirstTime);
  }
}

void CNHSensorManager::CSensorStatusUpdaterFile::SetCurrentTime(double dTime) {
  _nFileLineNum = _pSensorFromFile->GetNearestLine(dTime,2);
}

boost::posix_time::ptime CNHSensorManager::SensorTimeToBoostTime(double dTime) const {

  boost::posix_time::time_duration Duration=boost::posix_time::microseconds((int64_t)(dTime*1000*1000));
  return _FirstBoostTime + Duration;
}



