#pragma once

#include "MovingObject.h"
#include "MOTracker.h"
#include "TrackerConfig.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/noncopyable.hpp>

class CMOTrackerViewModel;
class CMOTrackerPresenter;

class CMOTrackerFramework : boost::noncopyable
{

public:
  CMOTrackerFramework(void);
  virtual ~CMOTrackerFramework(void);

  void Run(const STrackerConfig &rConfig);

  void FinishProgram() {
    _bRunTrackerLoop = false;
  }
  void ToggleWaitMode();
  void ProcOneFrame();
  void ResetAllTrack();
  void ToggleTrackMode();
  bool IsLogMode() const {
    return _bLogMode;
  }

  boost::shared_ptr<const CMOTracker> GetTracker() const {return _pTracker;}

protected:

  virtual boost::shared_ptr<CMOTracker> MakeTracker() = 0;
  virtual void MakePresentors(std::vector<boost::shared_ptr<CMOTrackerPresenter> > &rvpPresentors) = 0;
  virtual boost::shared_ptr<CLRFGrabber> MakeGrabber() = 0;

  bool _bLogMode;
  boost::shared_ptr<CMOTracker> _pTracker;

private:
  std::vector<boost::shared_ptr<CMOTrackerPresenter> > _vpPresentors;
  volatile bool _bRunTrackerLoop;
  boost::shared_ptr<CLRFGrabber> _pGrabber;
  CTrackerHistory _CurrentResult;

  bool _bWaitMode;
  bool _bPerformTrack;
};


class CMOTrackerView {
public:

  CMOTrackerView(boost::shared_ptr<CMOTrackerViewModel> pViewModel) {
    _pViewModel = pViewModel;
  }
  virtual ~CMOTrackerView() {}
  virtual void Update() = 0;

  void SetViewModel(boost::shared_ptr<CMOTrackerViewModel> pViewModel) {
    _pViewModel = pViewModel;
  }

protected:
  boost::shared_ptr<CMOTrackerViewModel> _pViewModel;
};

class CMOTrackerPresenter;
class CMOTrackerViewModel {
public:

  CMOTrackerViewModel(CMOTrackerPresenter* pPresenter) {
    _pPresenter = pPresenter;
  }
  virtual ~CMOTrackerViewModel() {}

  virtual void SetCurrentResult(const CTrackerHistory &rResult) = 0;

  CMOTrackerPresenter* GetPresenter() {
    return _pPresenter;
  }
  const CMOTrackerPresenter* GetPresenter() const{
    return _pPresenter;
  }
  

protected:

  CMOTrackerPresenter* _pPresenter;

};

class CMOTrackerPresenter {

public:
  CMOTrackerPresenter(CMOTrackerFramework *pModel) {
    _pModel = pModel;
  }
  virtual ~CMOTrackerPresenter() {}

  virtual void DoLoopProc(const CTrackerHistory &rHistory) = 0;

  void ToggleWaitMode();
  void ProcOneFrame();
  void ResetAllTrack();
  void ToggleTrackMode();

  virtual void ShowPreviousResult(){}
  virtual void ShowNextResult(){}

  bool IsLogMode() const {
    return _pModel->IsLogMode();
  }

  virtual void ProcKeyCode(int nKey) {};

protected:

  CMOTrackerFramework* _pModel;

  std::vector<boost::shared_ptr<CMOTrackerView> > _vpViews;
  std::vector<boost::shared_ptr<CMOTrackerViewModel> > _vpViewModels;
};


class CDoNothingPresenter: public CMOTrackerPresenter {

public:
  CDoNothingPresenter(CMOTrackerFramework *pModel) : CMOTrackerPresenter(pModel) {
  }
  virtual ~CDoNothingPresenter() {}

  virtual void DoLoopProc(const CTrackerHistory &rHistory) {
    Sleep(100);
  }
};


