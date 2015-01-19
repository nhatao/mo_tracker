#pragma once

#include "LRFSource.h"
#include "CascadedCoords.h"
#include "CascadedVec.h"
#include <boost/date_time/posix_time/posix_time.hpp>

class CLaserData : boost::noncopyable
{
public:
  CLaserData(const SLRFProperty& prop, const boost::posix_time::ptime& rTime, boost::shared_ptr<CCascadedCoords> pLRFCo) {
    _Property = prop;
    _Time = rTime;
    _pLRFCo = pLRFCo;

    using namespace boost::posix_time;
    ptime epoch(boost::gregorian::date(1970,1,1));
    _dTime = (rTime - epoch).total_microseconds()/(1000.0*1000.0);
  }
  virtual ~CLaserData(void) {
  }

  //dMultiFactor倍して代入．元のデータが[m]単位なら1000, [cm]単位なら10
  template<class it>
  void Set2DData(it pBegin, it pEnd, double dMultiFactor=1.0) {
    _vpPoints.erase(_vpPoints.begin(), _vpPoints.end());
    _vdRawData.erase(_vdRawData.begin(), _vdRawData.end());
    int n=0;
    for (it pCur = pBegin; pCur != pEnd; ++pCur, ++n) {
      double dRange = (*pCur) * dMultiFactor; 
      if ((dRange <= _Property._dMaxRange) && (dRange >= _Property._dMinRange) ) {
        double dRad = _Property._dFirstAngle.get() + _Property._dReso.get()*n;
        double x = dRange * cos(dRad);
        double y = dRange * sin(dRad);
        _vpPoints.push_back(CCascadedVec::MakeCasVec(x, y, 0.0, _pLRFCo));
        _vdRawData.push_back(dRange);
      }
      else {
        _vpPoints.push_back(CCascadedVec::MakeCasVec(0.0, 0.0, 0.0, _pLRFCo));
//        _vdRawData.push_back(dRange);
        _vdRawData.push_back(0); //temp 昔の仕様
      }
    }
    if (_vpPoints.size() != (size_t)_Property._nElemNum) {
      std::ostringstream oss;
      oss << __FUNCTION__ << " _vpPoints.size()=" << _vpPoints.size();
      oss << " _Property._nElemNum=" << _Property._nElemNum;
      throw oss.str().c_str();
    }
  }

  boost::shared_ptr<const CCascadedCoords> GetCo() const {
    return _pLRFCo;
  }

  const boost::posix_time::ptime &GetPosixTime() const {
    return _Time;
  }
  double GetTime() const {
    return _dTime;
  }
  const SLRFProperty& GetProperty() const {
    return _Property;
  }
  const std::vector<double> &GetRawData() const {
    return _vdRawData;
  }
  const std::vector<boost::shared_ptr<CCascadedVec> > &GetPoints() const {
    return _vpPoints;
  }

  boost::shared_ptr<CLaserData> Clone() const {
    auto *pNew = new CLaserData(_Property, _Time, _pLRFCo);
    pNew->_vdRawData = _vdRawData;
    pNew->_vpPoints = _vpPoints;
    return boost::shared_ptr<CLaserData>(pNew);
  }

protected:

  boost::posix_time::ptime _Time;
  double _dTime;
  boost::shared_ptr<CCascadedCoords> _pLRFCo;
  SLRFProperty _Property;
  std::vector<double> _vdRawData;
  std::vector<boost::shared_ptr<CCascadedVec> > _vpPoints;
};

