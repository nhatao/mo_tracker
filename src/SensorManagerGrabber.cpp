
#include "StdAfx_MOTracking.h"
#include "SensorManagerGrabber.h"
#include "NHSensorFromFile.h"
#include "URGSource.h"
#include "LMS100Source.h"
#include "ColorCout.h"
#include <boost/filesystem.hpp>
#include "LRFSensor.h"

using namespace std;


void CIPRobotStatus::WriteToLogImpl(std::ostream &out, int nFrameNum) {

  out << _dX << " " << _dY << " " << _dYaw << " "; 
  out << _dPitch << " " <<  _dRoll << " " << _dVel << " " << _dWVel;
}
void CIPRobotStatus::ReadFromLogImpl(std::istream &in) {

  in >> _dX >> _dY >> _dYaw;
  in >> _dPitch >> _dRoll;
  in >> _dVel >> _dWVel;
}


CSensorManagerGrabber::CSensorManagerGrabber(void) {
  _nOdmID = -1;
  _bLogMode = false;
}
CSensorManagerGrabber::~CSensorManagerGrabber(void) {}

class CNHSensorFromFile2 : public CNHSensorFromFile<CLRFStatus> {
public:
  CNHSensorFromFile2(const SLRFProperty &rP, const std::string &rsName) : CNHSensorFromFile<CLRFStatus>(rsName) {
    _Property = rP;
  };
  virtual ~CNHSensorFromFile2(void) {};
  virtual boost::shared_ptr<INHSensorStatus> MakeSensorStatus() const {
    return boost::shared_ptr<INHSensorStatus>(new CLRFStatus(_Property, this));
  };
protected:
  SLRFProperty _Property;
};

void CSensorManagerGrabber::Initialize(const STrackerConfig &Config) {

  _CurrentConfig = Config;
  _pManager = boost::shared_ptr<CNHSensorManager>(new CNHSensorManager);
  _vnSensorIDs.clear();

  if (_CurrentConfig._nMode == 0){
    cout << "Init File" << endl;
    if (_CurrentConfig._vsFileName.size() != _CurrentConfig._vLRFCos.size()) {
      ESStreamException ess; ess << "Size Unmatch: " << _CurrentConfig._vsFileName.size() << "!=" << _CurrentConfig._vLRFCos.size();
      throw ess;
    }
    if (_CurrentConfig._vsFileName.empty()) {
      throw std::logic_error("FileName empty");
    }
    boost::posix_time::ptime pDirTime(boost::gregorian::date(1970,1,1));
    _pManager->SetToFileMode(pDirTime);
    size_t i=0;
    for (auto itS = _CurrentConfig._vsFileName.begin(); itS != _CurrentConfig._vsFileName.end(); ++itS, ++i) {

      SLRFProperty p = _CurrentConfig._vLogLRFProperty.at(i);
      auto pLRF = boost::shared_ptr<CNHSensorFromFile2>(new CNHSensorFromFile2(p,"URG"));
      pLRF->SetFile(*itS, (int)round(p._dScanTime*1000));
      unsigned int n = _pManager->AddSensorWithCallback(pLRF, 1000, false, 
        boost::bind(&CSensorManagerGrabber::ObtainData, this, _1));
      _vnSensorIDs.push_back(n);
    }
    _pManager->SetFirstTime(_CurrentConfig._dStartTime);

    if (_CurrentConfig._sOdmFileName != "") {

      auto pOdm = boost::shared_ptr<CNHSensorFromFile<CIPRobotStatus> >(new CNHSensorFromFile<CIPRobotStatus>("Odometry"));
      pOdm->SetFile(_CurrentConfig._sOdmFileName, 10);
      _nOdmID = (int)_pManager->AddSensor(pOdm, 1000, false);
      cout << "Odometry file: " << _CurrentConfig._sOdmFileName << endl;
    }
    else {
      cout << "No Odometry file."<< endl;
    }
    _bLogMode = true;
  }
  else if (_CurrentConfig._nMode == 1) {
    cout << "Init URG" << endl;
    if (_CurrentConfig._vnPorts.size() != _CurrentConfig._vLRFCos.size()) {
      ESStreamException ess; ess << "Size Unmatch: " << _CurrentConfig._vnPorts.size() << "!=" << _CurrentConfig._vLRFCos.size();
      throw ess;
    }
    for (size_t i = 0; i < _CurrentConfig._vnPorts.size(); ++i) {
      boost::shared_ptr<CURGSource> pURG(new CURGSource());
      ostringstream oss; oss << "URG" << i;
      boost::shared_ptr<CLRFSensor> pURGSensor(new CLRFSensor(oss.str()));
      pURG->Init(_CurrentConfig._vnPorts[i]);
      pURGSensor->SetLRF(pURG);
//      unsigned int n = _pManager->AddSensor(pURGSensor, 1000, _CurrentConfig._bWriteLRFLog, 25);
      unsigned int n = _pManager->AddSensorWithCallback(pURGSensor, 1000, false, 
        boost::bind(&CSensorManagerGrabber::ObtainData, this, _1), 25);

      _vnSensorIDs.push_back(n);
    }
    _pManager->SetToRealTimeMode();
  }
  else if (_CurrentConfig._nMode == 2) {
    if ( (_CurrentConfig._vnPorts.size() != _CurrentConfig._vLRFCos.size()) || (_CurrentConfig._vsAddrName.size() != _CurrentConfig._vLRFCos.size())) {
      ESStreamException ess;
      ess << "Size Unmatch: Addr=" << _CurrentConfig._vsAddrName.size();
      ess << " Port=" << _CurrentConfig._vnPorts.size() << " CO=" << _CurrentConfig._vLRFCos.size();
      throw ess;
    }
    if ( (_CurrentConfig._vnFogMode.size() != _CurrentConfig._vLRFCos.size()) || (_CurrentConfig._vnNPalse.size() != _CurrentConfig._vLRFCos.size())) {
      ESStreamException ess;
      ess << "Size Unmatch: Fog=" << _CurrentConfig._vnFogMode.size();
      ess << " NPalse=" << _CurrentConfig._vnNPalse.size() << " CO=" << _CurrentConfig._vLRFCos.size();
      throw ess;
    }

    for (size_t i=0; i<_CurrentConfig._vLRFCos.size(); ++i) {

      const string &rsAddr = _CurrentConfig._vsAddrName[i];
      int nPort = _CurrentConfig._vnPorts[i];

      cout << "Opening LMS: Addr=" << rsAddr << " Port=" << nPort << endl;
      if (rsAddr == "") {
        throw std::logic_error("Empty AddrName");
      }
      if (nPort == 0) {
        throw std::logic_error("Port=0");
      }

      cout << " Fog:" << _CurrentConfig._vnFogMode[i] << " Palse:" << _CurrentConfig._vnNPalse[i] << endl;
      ostringstream oss; oss << "LMS" << i;

      try {

        boost::shared_ptr<CLMS100Source> pLRF(new CLMS100Source());
        boost::shared_ptr<CLRFSensor> pLRFSensor(new CLRFSensor(oss.str()));
        pLRF->Init(rsAddr, nPort, CLMS100Source::e025DegAt25Hz);
        pLRF->SetFogFilterMode(_CurrentConfig._vnFogMode[i] != 0);
        pLRF->SetNto1FilterMode(_CurrentConfig._vnNPalse[i] != 0);

        pLRFSensor->SetLRF(pLRF);
        unsigned int n = _pManager->AddSensor(pLRFSensor, 1000, _CurrentConfig._bWriteLRFLog, 40);
        _vnSensorIDs.push_back(n);
      }
      catch (...) {
        throw;
      }
      /*
      catch (std::exception &e) {
        throw;
      }
      */
    }
  }
  else {
    ESStreamException ess;
    ess << "Unknwon Mode: " << _CurrentConfig._nMode;
    throw ess;
  }

  _dLastTime = -DBL_MAX;
}


void CSensorManagerGrabber::Tick() {
  vector<const INHSensorStatus *> vpStates;
  _pManager->GetLatestStatusGroup(_vnSensorIDs, vpStates);

}

void CSensorManagerGrabber::Update(const CSubject& rSubject, const std::string &sMsg){

  try {
    Tick();
  }
  catch (std::exception &e) {
    cout << __FUNCTION__ << " " << e.what() << endl;
  }
}

void CSensorManagerGrabber::Start() {

  _pManager->RunManager();
  Tick();

}
void CSensorManagerGrabber::Stop() {

  _pManager->StopManager();
}

void CSensorManagerGrabber::ObtainData(boost::shared_ptr<const INHSensorStatus> pStatus) {

  const CLRFStatus* pLRFStatus = (const CLRFStatus*)(pStatus.get());
  _dLastTime = pLRFStatus->GetMeasuredTime();
  SLRFProperty prop = pLRFStatus->GetProperty();
  const auto &rCo = _CurrentConfig._vLRFCos[0];

  boost::shared_ptr<CCascadedCoords> pOdmCo;
  if (_nOdmID > 0) {
    const CIPRobotStatus *pOdm = (const CIPRobotStatus*)(_pManager->GetNearestStatus(_nOdmID, pLRFStatus->GetMeasuredTime()));
//    cout << pOdm->GetMeasuredTime() << " " << pOdm->GetX() << " " << pOdm->GetY() << " " << pOdm->GetYaw() << endl;
    pOdmCo = CCascadedCoords::MakeCasCoords(
      pOdm->GetX(), pOdm->GetY(), 0, pOdm->GetYaw(), pOdm->GetPitch(), pOdm->GetRoll(),  "odm0");
  }
  else {
    pOdmCo = CCascadedCoords::MakeCasCoords(0, 0, 0, 0, 0, 0, "odm0");
  }
  boost::shared_ptr<CCascadedCoords> pLRFCo = CCascadedCoords::MakeCasCoords(
      rCo.GetPos()(0), rCo.GetPos()(1), 0, rCo.GetRotation(), 0, 0, "LRF0", pOdmCo);

  double dTime = pLRFStatus->GetMeasuredTime();
  auto pTime = _pManager->SensorTimeToBoostTime(dTime);

  boost::shared_ptr<CLaserData> pData(new CLaserData(prop, pTime, pLRFCo));
  pData->Set2DData(pLRFStatus->GetBuf().begin(), pLRFStatus->GetBuf().end());


  boost::mutex::scoped_lock lk(_DataMutex);
  _pData = pData;

  std::vector<boost::shared_ptr<const CLaserData>> vData;
  vData.push_back(_pData);
  SetLatestResult(vData);


}
