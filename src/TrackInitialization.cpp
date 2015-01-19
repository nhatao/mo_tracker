#include "StdAfx_MOTracking.h"
#include "TrackInitialization.h"

using namespace std;


bool CTrackInitializationWithProperty::IsPointValid(const BoostVec &rV, size_t nLRFNo) const {

  for (auto it=_vInvalidRegions.begin(); it!=_vInvalidRegions.end(); ++it) {
    if ( (it->first  == nLRFNo) && (it->second.IsPointInner(rV(0), rV(1)))){
      return false;
    }
  }
  for (auto it=_vValidRegions.begin(); it!=_vValidRegions.end(); ++it) {
    if (it->first  == nLRFNo) {
      if ((it->second.GetSize() == 0) || (it->second.IsPointInner(rV(0), rV(1)))) return true;
      else return false;
    }
  }
  return true;
}
