#include "StdAfx_MOTracking.h"
#include "CascadedVec.h"
#include <iomanip>

boost::shared_ptr<CCascadedVec> CCascadedVec::MakeCasVec(double x, double y, double z, boost::shared_ptr<const CCascadedCoords> pParent) {

  boost::shared_ptr<CCascadedVec> p(new CCascadedVec(x,y,z,pParent));
  pParent->AddChild(p);
  return p;
}

CCascadedVec::CCascadedVec(double x, double y, double z, boost::shared_ptr<const CCascadedCoords> pParent) {

  _LocalVec.resize(3);
  _LocalVec(0) = x; _LocalVec(1) = y; _LocalVec(2) = z;
  _pParent = pParent;

  _WorldVec.resize(3);
  _bChanged = true;
}

CCascadedVec::~CCascadedVec(void){}

const BoostVec &CCascadedVec::GetLocalVec() const {
  return _LocalVec;
}
const BoostVec &CCascadedVec::GetWorldVec() const {

  if (_bChanged || _pParent->Changed()) {
    _pParent->VectorLocalToGlobal(_LocalVec, _WorldVec);
    _bChanged = false;
  }
  return _WorldVec;
}


void CCascadedVec::SetWorld(double x, double y, double z) {
  _WorldVec(0) = x;
  _WorldVec(1) = y;
  _WorldVec(2) = z;
  _bChanged = false;
  _pParent->VectorGlobalToLocal(_WorldVec, _LocalVec);
}

void CCascadedVec::SetLocal(double x, double y, double z) {

  _LocalVec(0) = x;
  _LocalVec(1) = y;
  _LocalVec(2) = z;
  _bChanged = true;
}
