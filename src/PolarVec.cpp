#include "StdAfx_MOTracking.h"
#include "PolarVec.h"

CPolarVec::CPolarVec(const BoostVec& rhs) {
  this->operator=(rhs);
}

CPolarVec& CPolarVec::operator=(const BoostVec& rhs) {
  double x=rhs(0);
  double y=rhs(1);
  _dDist = sqrt(x*x+y*y);
  _Angle.set(atan2(y, x));
  return *this;
}
