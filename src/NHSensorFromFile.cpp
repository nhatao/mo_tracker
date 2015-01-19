#include "StdAfx_MOTracking.h"
#include "NHSensorFromFile.h"


using namespace std;


bool CNHSensorFromFileImpl::UpdateStatusImpl(INHSensorStatus *pStatus) const {
  MeasureNearestTime(*pStatus, pStatus->GetMeasuredTime());
  return true;
}

void CNHSensorFromFileImpl::Init() {

  if (_vdTimes.empty()) {
    ESStreamException ess;
    ess << "CNHSensorFromFile Not Initialized!!";
    throw ess;
  }
}

void CNHSensorFromFileImpl::MeasureNthStatus(INHSensorStatus &rStatus, size_t nTarget) const {

  if (nTarget >= _vdTimes.size()) {
    ESStreamException ess;
    ess << "Measure Nth Status Over!" << nTarget << " " << _vdTimes.size();
    throw ess;
  }
  SetNthStatus(rStatus, nTarget);
}

size_t CNHSensorFromFileImpl::GetNearestLine(double dTime, int nStatus) const{

  //dTimeに最も近いデータ番号を取得
  if (_vdTimes.empty()) {
    ESStreamException ess; ess << "_vdTimes Empty " << GetSensorName();
    throw ess;
  }

  size_t nTarget;
  size_t nFirst = 0;
  size_t nLast = _vdTimes.size()-1;
  if (_vdTimes[nFirst] > dTime) nTarget = nFirst;
  else if (_vdTimes[nLast] < dTime) nTarget = nLast;
  else {
    while (true) {
      if (nLast-nFirst==1) {
        break;
      }
      size_t nCenter = (nFirst+nLast)/2;
      if (_vdTimes[nCenter] > dTime) {
        nLast = nCenter;
      }
      else {
        nFirst = nCenter;
      }
    } //while

    if (nStatus == 0) {
      if ( abs(_vdTimes[nFirst]-dTime) < abs(_vdTimes[nLast]-dTime) ) {
        nTarget = nFirst;
      }
      else {
        nTarget = nLast;
      }
    }
    else if (nStatus == 1) {
      nTarget = nFirst;
    }
    else if (nStatus == 2) {
      nTarget = nLast;
    }
  }
  return nTarget;
}

void CNHSensorFromFileImpl::MeasureNearestTime(INHSensorStatus &rStatus, double dTime, int nStatus) const {
  size_t nTarget = GetNearestLine(dTime, nStatus);
  SetNthStatus(rStatus, nTarget);
  if ( rStatus.GetMeasuredTime() != _vdTimes[nTarget] ) {
    cout << "Measure time broken " << rStatus.GetMeasuredTime() << " -> " << _vdTimes[nTarget] << endl;
  }
}


bool CNHSensorFromFileImpl::Measure() {
  //do nothing;
  return true;
}

void CNHSensorFromFileImpl::GetDataforGivenPeriod(std::vector<boost::shared_ptr<INHSensorStatus> > &raResult, double dTimeBegin, double dTimeEnd) 
{
  size_t nTargetBegin = GetNearestLine(dTimeBegin, 2);
  size_t nTargetEnd   = GetNearestLine(dTimeEnd, 1);

  raResult.clear();
  for (size_t i=nTargetBegin; i<nTargetEnd; ++i) {

    auto p = MakeSensorStatus();
    SetNthStatus(*p, i);
    raResult.push_back(p);
  }
}
