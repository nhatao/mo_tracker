#include "StdAfx_MOTracking.h"
#include "MOTrackerFramework.h"
#include "OutputDebugStream.h"
#include "FLTKPresenter.h"
#include "SensorManagerGrabber.h"
#include "JPDATracker.h"
#include "SJPDATrackerConfig.h"
#include <boost/program_options.hpp>

#ifndef _MSC_VER
#include <unistd.h>
#include "ROSGrabber.h"
#include "ROSMessageSender.h"
#else
#include <mmsystem.h>
#endif

using namespace std;

#ifndef _MSC_VER
class CRosPresentor : public CMOTrackerPresenter {
public:
  CRosPresentor(CMOTrackerFramework *pModel) : CMOTrackerPresenter(pModel) {}
  virtual ~CRosPresentor() {
    ros::shutdown();
  }

  virtual void DoLoopProc(const CTrackerHistory &rHistory) {
    ros::spinOnce();
    if (!ros::ok()) {
      _pModel->FinishProgram();
    }
    usleep(100*1000);
  }
};
#endif

class CMOTrackerApp : public CMOTrackerFramework {
public:

  CMOTrackerApp() : CMOTrackerFramework() {}
  virtual ~CMOTrackerApp() {}

  void Start(int argc, char **argv) {
    boost::program_options::options_description opt("MOTracker");
    opt.add_options()
      ("inifile,f", boost::program_options::value<string>()->default_value("config/ini/DefaultParam.ini"), "Inifile Name");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(opt).allow_unregistered().run(), vm);
    boost::program_options::notify(vm);
    
    std::string sIniFileName = vm["inifile"].as<string>();
    cout << "Ini file: " << sIniFileName << endl;
    _Config.LoadFromIni(sIniFileName);

    _argc = argc;
    _argv = argv;

    Run(_Config);
  }

protected:

  //trackerを作成
  virtual boost::shared_ptr<CMOTracker> MakeTracker() {
    return boost::shared_ptr<CMOTracker> (new CJPDATracker(_Config));
    /*
    if (_Config._nTrackerType == 0) {
      cout << "Initializing JPDA Tracker" << endl;
      return boost::shared_ptr<CMOTracker> (new CJPDATracker(_Config));
    }
    else if (_Config._nTrackerType == 1) {
      cout << "Initializing Original SJPDAF Tracker" << endl;
      return boost::shared_ptr<CMOTracker> (new COriginalSJPDAF(_Config));
    }
    else if (_Config._nTrackerType == 2) {
      cout << "Initializing MHT" << endl;
      return boost::shared_ptr<CMOTracker> (new CMHT(_Config));
    }
    */
  }
  virtual void MakePresentors(std::vector<boost::shared_ptr<CMOTrackerPresenter> > &rvpPresentors) {
    CFLTKPresenter *pFLTKPresenter;
    if (_Config._bDrawViewer) {
      pFLTKPresenter = new CFLTKPresenter(this);
      pFLTKPresenter->Initialize(_Config);
      rvpPresentors.push_back(boost::shared_ptr<CMOTrackerPresenter>(pFLTKPresenter));
    }
    else {
      //presentorがないと無駄な高速ループになってしまうのを防ぐ
      rvpPresentors.push_back(boost::shared_ptr<CMOTrackerPresenter>(new CDoNothingPresenter(this)));
    }
    if (_Config._nMode == 3) {
#ifdef _MSC_VER
      throw std::logic_error("ROS Mode is not supported on Windows.");
#else
      rvpPresentors.push_back(boost::shared_ptr<CMOTrackerPresenter>(new CRosPresentor(this)));
#endif
    }
  }
  virtual boost::shared_ptr<CLRFGrabber> MakeGrabber() {

    if (_Config._nMode == 3) {
#ifdef _MSC_VER
      throw std::logic_error("ROS Mode is not supported on Windows.");
#else
      ros::init(_argc, _argv, "mo_tracker_node", ros::init_options::AnonymousName);
      auto *pGrabber = new CROSGrabber();
      pGrabber->SetWaitForTransform(_Config._bWaitForTransform);
      pGrabber->ShowDebug();
      auto *pSender = new CROSMessageSender();
      pSender->Init();
      _pTracker->AddObserver(boost::shared_ptr<CObserver>(pSender));
      return boost::shared_ptr<CLRFGrabber>(pGrabber);
#endif
    }
    else {
      boost::shared_ptr<CSensorManagerGrabber> pGrabber(new CSensorManagerGrabber());
      pGrabber->Initialize(_Config);
      _pTracker->AddObserver(pGrabber);
      return pGrabber;
    }
  }

  SJPDATrackerConfig _Config;
  int _argc;
  char **_argv;

};


int main(int argc, char **argv) {

  std::ios_base::sync_with_stdio(false);

#ifdef _MSC_VER
  TIMECAPS timercaps;
  ZeroMemory(&timercaps, sizeof(timercaps));
  MMRESULT mmresult = timeGetDevCaps(&timercaps, sizeof(TIMECAPS));
  auto _uPeriod = timercaps.wPeriodMin;
  mmresult = timeBeginPeriod(_uPeriod);
  dout.SetMode(1);
#else
  dout.SetMode(0);
#endif

  try {
    CMOTrackerApp TrackerApp;
    TrackerApp.Start(argc, argv);
  }
  catch (boost::exception &e) {
    cout << diagnostic_information(e) << endl;
  }
  catch (std::exception &e) {
    cout << e.what() << endl;
  }

  return 0;
}

