#include "StdAfx_MOTracking.h"
#include "MOTrackerFramework.h"
#include "TrackerConfig.h"
#include "ColorCout.h"
#include "TrackerLogger.h"

using namespace std;


CMOTrackerFramework::CMOTrackerFramework(void)
{
  _bLogMode = false;
  _bWaitMode = false;
  _bPerformTrack = true;
}


CMOTrackerFramework::~CMOTrackerFramework(void){}

void CMOTrackerFramework::Run(const STrackerConfig &rConfig) {

  try {

    //コンポーネント作成
    _pTracker = MakeTracker();
    MakePresentors(_vpPresentors);
    _pGrabber = MakeGrabber();
    _bLogMode = _pGrabber->IsLogMode();

    _CurrentResult.set_capacity(rConfig._nHistorySize);
    _pTracker->SetCapacity(rConfig._nHistorySize);
    CMovingObject::SetHistoryMaxNum(rConfig._nHistorySize);

    if (rConfig._bWriteLRFLog) {
      if (!boost::filesystem::exists("log")) {
        boost::filesystem::create_directory("log");
      }
      ostringstream oss;
      string sDirName =  boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
      oss << "log/" << sDirName;
      _pGrabber->SetLogDir(oss.str());
    }
    if (rConfig._bWriteTrackerLog) {
      if (!boost::filesystem::exists("TrackerLog")) {
        boost::filesystem::create_directory("TrackerLog");
      }
      ostringstream oss;
      string sDirName =  boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
      oss << "TrackerLog/" << sDirName << ".dat";
      auto *pLogger = new CTrackerLogger();
      pLogger->Init(oss.str());
      _pTracker->AddObserver(boost::shared_ptr<CObserver>(pLogger));
    }

    if (_bLogMode && !rConfig._bAuto) {
      _bWaitMode = true;
    }
    _pTracker->SetWaitViewer(_bWaitMode);

    _pGrabber->Start();   
    _pTracker->RunThread(_pGrabber);
    
    ProcOneFrame();

    _bRunTrackerLoop = true;
    while (_bRunTrackerLoop) {
      if (!(_bLogMode && _bWaitMode)) {
        _pTracker->Tick();
      }
      _pTracker->GetLatestResults(_CurrentResult);
      for (auto it=_vpPresentors.begin(); it!=_vpPresentors.end(); ++it) {
        (*it)->DoLoopProc(_CurrentResult);
      }
      Sleep(0);
    }
    cout << "finishing program." << endl;
    _pTracker->FinishThread();
    _pGrabber->Stop();
  }
  catch (std::exception &e) {
    cout << e.what() << endl;
  }
}

void CMOTrackerFramework::ProcOneFrame() {

  if (_bLogMode) {
    _pTracker->Tick();
  }
}


void CMOTrackerFramework::ToggleWaitMode() {

  if (_bLogMode) {
    _bWaitMode = !_bWaitMode;
    _pTracker->SetWaitViewer(_bWaitMode);
  }
}


void CMOTrackerPresenter::ToggleWaitMode() {

  _pModel->ToggleWaitMode();
}

void CMOTrackerPresenter::ProcOneFrame() {

  _pModel->ProcOneFrame();
}


void CMOTrackerPresenter::ResetAllTrack() {
  
  _pModel->ResetAllTrack();
}


void CMOTrackerPresenter::ToggleTrackMode() {
  
  _pModel->ToggleTrackMode();


}

void CMOTrackerFramework::ResetAllTrack() {

  _pTracker->ResetAll();
  
}


void CMOTrackerFramework::ToggleTrackMode() {

  _bPerformTrack = !_bPerformTrack;
  _pTracker->SetPerformTracking(_bPerformTrack);
}
