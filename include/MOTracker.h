#pragma once

#include "LaserData.h"
#include "MovingObject.h"
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include "observer.h"
#include "mt_ofstream.h"

typedef std::pair<
            boost::shared_ptr<const STrackerResult>,
            std::vector<boost::shared_ptr<const CLaserData>>> 
            CTrackerResult;

typedef boost::circular_buffer<boost::shared_ptr<CTrackerResult>> CTrackerHistory;

typedef boost::circular_buffer< std::vector<boost::shared_ptr<const CLaserData>>> LaserDataBuffer;

//in  : std::vector<boost::shared_ptr<CLaserData> >
//out : boost::shared_ptr<STrackerResult>

class CLRFGrabber;

class CMOTracker : public CSubject
{
public:
  CMOTracker(void);
  virtual ~CMOTracker(void);

  void RunThread(boost::shared_ptr<CLRFGrabber> pGrabber);
  void FinishThread();
  void GetLatestResults(CTrackerHistory &rResults);
  void GetLatestResult(CTrackerResult &rResult) const;

  void SetWaitViewer(bool b) {
    _bWaitViewer = b;
  }
  bool GetWaitViewer() const {return _bWaitViewer;}
  void SetCapacity(int n) {
    _History.set_capacity(n);
  }
  virtual boost::shared_ptr<const STrackerResult> Proc(const LaserDataBuffer &rvLasers) = 0;

  void operator()();

  void Tick();

  virtual void ResetAll() = 0;
  virtual void SetPerformTracking(bool bPerformTrack) = 0;
protected:

  CTrackerResult _LatestResult;

  mutable boost::recursive_mutex _DataWaitMutex;
  boost::condition_variable_any  _DataWaitCondition;

  volatile bool _bWaitViewer;
  boost::recursive_mutex _ViewerWaitMutex;
  boost::condition_variable_any  _ViewerWaitCondition;

  volatile bool _bRunTrackerLoop;
  boost::shared_ptr<boost::thread> _pThr;
  boost::shared_ptr<CLRFGrabber> _pGrabber;
  boost::shared_ptr<boost::promise<CTrackerResult>> _pPromise;
  CTrackerHistory _History;

  volatile bool _bCriticalError;
};

class CLRFGrabber {
public:


  CLRFGrabber() {
    _Buffer.set_capacity(30);
  };
  virtual ~CLRFGrabber();

  void SetLogDir(const std::string &rsLogDir);

  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual bool IsLogMode() const {return false;}

  void GetResultBuffer(LaserDataBuffer &Buffer);

protected:

  //子クラスはデータを受信したらこの関数を呼ぶ
  void SetLatestResult(std::vector<boost::shared_ptr<const CLaserData> > &rvLaserData);

  boost::mutex _BufferMutex;
  LaserDataBuffer _Buffer;

  std::vector<boost::shared_ptr<mt_ofstream> > _vpLRFFiles;
  boost::shared_ptr<mt_ofstream> _pOdmFile;
  std::string _sLogDir;

};
