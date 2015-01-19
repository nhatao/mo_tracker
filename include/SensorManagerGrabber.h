#pragma once
#include "MOTracker.h"
#include "TrackerConfig.h"
#include <boost/thread/mutex.hpp>
#include "NHSensorManager.h"

class CIPRobotStatus :public INHSensorStatus 
{
public:
  CIPRobotStatus(const INHSensor *pParent) : INHSensorStatus(pParent) {
  }
  CIPRobotStatus(double dTime) : INHSensorStatus(dTime) {
  }
  virtual ~CIPRobotStatus() {}

  double GetX() const {return _dX;}
  double GetY() const {return _dY;}
  double GetRoll() const {return _dRoll;}
  double GetPitch() const {return _dPitch;}
  double GetYaw() const {return _dYaw;}
  double GetVel() const {return _dVel;}
  double GetWVel() const {return _dWVel;}

protected:

  double _dX;
  double _dY;
  double _dRoll;
  double _dPitch;
  double _dYaw;
  
  double _dVel;
  double _dWVel;

  virtual void WriteToLogImpl(std::ostream &out, int nFrameNum);
  virtual void ReadFromLogImpl(std::istream &in);

};

//LRF単体しか対応してない
class CSensorManagerGrabber :
  public CLRFGrabber, public CObserver
{
public:
  CSensorManagerGrabber(void);
  virtual ~CSensorManagerGrabber(void);

  void Initialize(const STrackerConfig &Config);

  virtual void Tick();
  virtual void Start();
  virtual void Stop();
  virtual bool IsLogMode() const {return _bLogMode;}

  virtual void Update(const CSubject& rSubject, const std::string &sMsg);

protected:

  STrackerConfig _CurrentConfig;
  boost::mutex _ConfigMutex;
  boost::shared_ptr<CNHSensorManager> _pManager;
  std::vector<unsigned int> _vnSensorIDs;
  double _dLastTime;

  CCoordinates2D _GlobalCo;

  void ObtainData(boost::shared_ptr<const INHSensorStatus> pStatus);

  boost::shared_ptr<CLaserData> _pData;
  boost::mutex _DataMutex;

  int _nOdmID;
  bool _bLogMode;
};

