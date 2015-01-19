#include "StdAfx_MOTracking.h"
#include "LRFSensor.h"
#include <boost/cast.hpp>
using namespace std;

CLRFStatus::CLRFStatus(int nPointNum, double dFirstDeg, double dReso, const INHSensor* pParent) : INHSensorStatus(pParent) {
  _Property._nElemNum = nPointNum;
  _Property._dFirstAngle.set_deg(dFirstDeg);
  _Property._dReso.set_deg(dReso);
  _vBuf.resize(nPointNum);
}

CLRFStatus::CLRFStatus(const SLRFProperty &rProperty, const INHSensor* pParent) : INHSensorStatus(pParent) {
  _Property = rProperty;
  _vBuf.resize(_Property._nElemNum);
}

CLRFStatus::CLRFStatus(const INHSensor* pParent) : INHSensorStatus(pParent) {
  cout << "CLRFStatus initilalized error! Default URG value used" << endl;
  _vBuf.resize(_Property._nElemNum);
}

CLRFStatus::~CLRFStatus() {
}

CLRFStatus& CLRFStatus::operator=(const CLRFStatus& rhs) {

  this->INHSensorStatus::operator=(rhs);
  _Property = rhs._Property;
  _vBuf = rhs._vBuf;
  return *this;
}


void CLRFStatus::WriteToLogImpl(std::ostream &out, int nFrameNum) {
  if (_vBuf.size() != _Property._nElemNum) {
    ESStreamException ess;
    ess << "WriteToLogImpl _vBuf.size() != _nLRFPointNum " << _vBuf.size() << "!=" << _Property._nElemNum;
    throw ess;
  }
  for (size_t i=0; i<_vBuf.size(); ++i) {
//    out << _pBuf[i] << " ";
    out << _vBuf[i] << " ";
  }
}

void CLRFStatus::ReadFromLogImpl(std::istream &in) {
  if (_vBuf.size() != _Property._nElemNum) {
    ESStreamException ess;
    ess << "ReadToLogImpl _vBuf.size() != _nLRFPointNum " << _vBuf.size() << "!=" << _Property._nElemNum;
    throw ess;
  }

  std::istringstream *piss = dynamic_cast<std::istringstream *>(&in);
  if (piss) {
    //デバッグ版だと重すぎて動かないのでこれを作った．
    string s = piss->str();
    const char* pBuf = s.c_str();
    const char* pCur = pBuf;
    const char* pEnd = pCur + strlen(pBuf);
//    cout <<  piss->str() << endl;

    for (auto it = _vBuf.begin(); it != _vBuf.end(); ++it) {
      char *pNext;
      long n = strtol(pCur, &pNext, 10);
      (*it) = (double)n;
      if (pNext == pEnd) {
        cout << "iss: " << piss->str() << endl;
        for (; it != _vBuf.end(); ++it) {
          (*it) = 0;
        }
        break;
      }
      ++pNext;
      pCur = pNext;
    }
  }
  else {
    for (auto it = _vBuf.begin(); it != _vBuf.end(); ++it) {
      if (in.fail() || (in.eof())) {
        (*it) = 0;
      }
      else {
        in >> (*it);
      }
    }
  }
}

CLRFSensor::CLRFSensor(void) : INHSensor("URG")
{
  _bHasHardwareClock = false;
}

CLRFSensor::CLRFSensor(const std::string& rsName) : INHSensor(rsName)
{
}
CLRFSensor::~CLRFSensor(void)
{
}
using namespace std;
void CLRFSensor::SetLRF(boost::shared_ptr<CLRFSource> pLRF) {
  _pLRF = pLRF;
  _Property = pLRF->GetProperty();
}

void CLRFSensor::Init() {
}

bool CLRFSensor::Measure() {
  _pLRF->RequestData();
  _pLRF->Measure();
  return true;
}

unsigned int CLRFSensor::GetMinUpdateTime() const {

  return 10; //TODO センサから分かるようにする方がいい
}

boost::shared_ptr<INHSensorStatus> CLRFSensor::MakeSensorStatus() const {

//  return boost::shared_ptr<INHSensorStatus>(new CLRFStatus(_nLRFPointNum, _pLRF->GetFirstDeg(), _pLRF->GetReso(), this));
  return boost::shared_ptr<INHSensorStatus>(new CLRFStatus(_Property, this));
}

bool CLRFSensor::UpdateStatusImpl(INHSensorStatus *pStatus) const {

  CLRFStatus* pURGStatus = boost::polymorphic_downcast<CLRFStatus*>(pStatus);
  if (_Property._nElemNum != pURGStatus->GetPointNum()) {
    ESStreamException ess;
    ess << "CLRFSensor::UpdateStatusImpl _nLRFPointNum != pURGStatus->GetPointNum()  " << _Property._nElemNum << " != " << pURGStatus->GetPointNum();
    throw ess;
  }
  const unsigned short* pBuf = GetLRF()->ReadData();
  for (auto it = pURGStatus->_vBuf.begin(); it != pURGStatus->_vBuf.end(); ++it, ++pBuf) {
    (*it) = (*pBuf);
  }

  return true;
}
