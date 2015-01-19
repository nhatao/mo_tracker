#include "StdAfx_MOTracking.h"
#include "TrackInitializationPassThrough.h"

using namespace std;


void CTrackInitializationPassThrough::Update(const LaserDataBuffer &rLog, std::vector<int> &vnExtracted) {

  if (rLog.empty()) return;
  const auto rvFirst = rLog.back();
  double dMinRange = rvFirst.front()->GetProperty()._dMinRange;
  double dMaxRange = rvFirst.front()->GetProperty()._dMaxRange;

  const vector<double> &rvRawData = rvFirst.at(0)->GetRawData();

  for (size_t i=0; i<rvRawData.size(); ++i) {
    if ((dMinRange < rvRawData[i]) && (rvRawData[i] < dMaxRange)) {
      vnExtracted.push_back((int)i);
    }
  }
}
