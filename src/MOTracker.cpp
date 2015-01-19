#include "StdAfx_MOTracking.h"
#include "MOTracker.h"
#include "ColorCout.h"

using namespace std;

CMOTracker::CMOTracker() {

  _bWaitViewer = false;
}


CMOTracker::~CMOTracker() {
  FinishThread();
}

void CMOTracker::FinishThread() {

  if (_pThr) {

    _bRunTrackerLoop = false;
    {
      boost::recursive_mutex::scoped_lock lk(_DataWaitMutex);
      _DataWaitCondition.notify_all();
    }
    {
      boost::recursive_mutex::scoped_lock lk(_ViewerWaitMutex);
      try {
        _ViewerWaitCondition.notify_all();
        if (_pPromise) {
          _pPromise->set_value(_LatestResult);
        }
      }
      catch (...){
      }
    }
    _pThr->join();
    _pThr.reset();
  }

}

void CMOTracker::RunThread(boost::shared_ptr<CLRFGrabber> pGrabber) {

  _pGrabber = pGrabber;
  _bRunTrackerLoop = true;
  _pThr.reset(new boost::thread( boost::ref(*this) ));
}

void CMOTracker::GetLatestResult(CTrackerResult &rResult) const  {

  boost::recursive_mutex::scoped_lock lk(_DataWaitMutex);
  if (_History.empty()) {
    throw std::logic_error("TrackerResult Empty");
  }
  rResult = *(_History.back());
}


void CMOTracker::Tick() {

  boost::recursive_mutex::scoped_lock lk(_ViewerWaitMutex);
  _ViewerWaitCondition.notify_all();
  if (_bWaitViewer) {
    boost::xtime xt;
    boost::xtime_get(&xt, boost::TIME_UTC_);
    xt.nsec += 10*1000*1000;
    _DataWaitCondition.timed_wait(lk, xt);
  }
}

void CMOTracker::GetLatestResults(CTrackerHistory &rResults) {

  boost::recursive_mutex::scoped_lock lk(_DataWaitMutex);
  rResults = _History;
}

void CMOTracker::operator()() {

#ifdef _MSC_VER
    HANDLE hThread = GetCurrentThread();
    if (!SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL)) {
      cout << __FUNCTION__  << " Priority Set NG" << endl;
    }
#endif

  int _nFrame2 = 0;
  mmtimer mt;
  double _dLastTime = 0;

  while (_bRunTrackerLoop) {

    LaserDataBuffer Buffer;
    _pGrabber->GetResultBuffer(Buffer);
    if (!Buffer.empty()) {

      ccout->SetColor(ColorCout::eGreen);
      boost::shared_ptr<const STrackerResult> pCurRes = Proc(Buffer);
      if (pCurRes) {
        try {
          {
            boost::recursive_mutex::scoped_lock lk(_DataWaitMutex);
            _LatestResult.first = pCurRes;
            _LatestResult.second = Buffer.back();
            boost::shared_ptr<CTrackerResult> pRes(new CTrackerResult);
            pRes->first = pCurRes;
            pRes->second = Buffer.back();
            _History.push_back(pRes);
            if (_bWaitViewer) {
              _DataWaitCondition.notify_all();
            }
          }
          {
            boost::recursive_mutex::scoped_lock lk(_ViewerWaitMutex);
            if (_bWaitViewer) {
              _ViewerWaitCondition.wait(lk);
            }
          }
          if (!_bRunTrackerLoop) break;

//          _pPromise->set_value(_LatestResult);

          Notify("TrackerUpdated");
  //        cout << "Frame: " << pCurRes->_nFrame << " ObjNum:" << pCurRes->_vpObjects.size() << endl;
          ++_nFrame2;

          if (_nFrame2 % 100 == 0) {
            cout << "TrackerThread FPS: " << _nFrame2/mt.elapsed() << endl;
          }
        } catch( boost::exception& e ) {
          cout << boost::diagnostic_information(e) << endl;
        }
      }
      else {
        Notify("TrackerUpdatedFailed");
        Sleep(1);
      }

    }
    else {
      //no data
      Sleep(1);
    }
  }

}


void CLRFGrabber::GetResultBuffer(LaserDataBuffer &Buffer) {
  boost::mutex::scoped_lock lk(_BufferMutex);
  Buffer = _Buffer;
  _Buffer.clear();


}

#include <boost/filesystem.hpp>

void CLRFGrabber::SetLatestResult(std::vector<boost::shared_ptr<const CLaserData> > &rvLaserData) {

  boost::mutex::scoped_lock lk(_BufferMutex);
  _Buffer.push_back(rvLaserData);


  if (rvLaserData.empty()) return;

  if (_sLogDir != "") {
//    const auto &rP = rvLaserData[0]->GetCo()->GetParent();
    const auto &rP = rvLaserData[0]->GetCo();
//    cout << "odm: " << rvLaserData[0]->GetCo()->GetStr() << endl;
//    cout << "parent: " << rvLaserData[0]->GetCo()->GetParent()->GetStr() << endl;
    (*_pOdmFile) << std::fixed << setprecision(2);
    (*_pOdmFile) << rP->GetX() << " " << rP->GetY() << " ";
    (*_pOdmFile) << std::fixed << setprecision(6);
    (*_pOdmFile) << rP->GetYaw() << " ";
    (*_pOdmFile) << "0 0 0 0 ";
    (*_pOdmFile) << rvLaserData[0]->GetTime() << endl;


    if (rvLaserData.size() > _vpLRFFiles.size()) {
      for (size_t j=_vpLRFFiles.size(); j<rvLaserData.size(); ++j) {
        ostringstream oss; 
        oss << _sLogDir << "/lrf" << j << ".dat";
        _vpLRFFiles.push_back(boost::shared_ptr<mt_ofstream>(new mt_ofstream(oss.str().c_str())));
      }
    }

    for (size_t i=0; i<rvLaserData.size(); ++i) {
      const auto & pLaser = rvLaserData[i];
      auto &pFile = _vpLRFFiles[i];

      for (auto it=pLaser->GetRawData().begin(); it!=pLaser->GetRawData().end(); ++it) {
        (*pFile) << (int)(*it) << " ";
      }
      (*pFile) << std::fixed << setprecision(6);
      (*pFile) << pLaser->GetTime() << endl;

    }
  }
}

void CLRFGrabber::SetLogDir(const std::string &rsLogDir) {

  _sLogDir = rsLogDir;
  if (_sLogDir != "") {

    boost::filesystem::path OurDir(rsLogDir);
    if (!boost::filesystem::exists(OurDir)) {
      boost::filesystem::create_directory(OurDir);
    }
    else {
      for (auto it=boost::filesystem::directory_iterator(OurDir); it!=boost::filesystem::directory_iterator(); ++it) {
        boost::filesystem::remove(*it);
      }
    }
    boost::filesystem::path pOdm = OurDir/"odm.dat";
    _pOdmFile.reset(new mt_ofstream(pOdm.string().c_str()));
    boost::filesystem::path pLRF0 = OurDir/"lrf0.dat";
    _vpLRFFiles.push_back(boost::shared_ptr<mt_ofstream>(new mt_ofstream(pLRF0.string().c_str())));

  }
}

CLRFGrabber::~CLRFGrabber () {
  cout << "LRFGrabber destructed!" << endl;
  boost::mutex::scoped_lock lk(_BufferMutex);
  _pOdmFile.reset();
  _vpLRFFiles.clear();

}