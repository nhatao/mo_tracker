#include "StdAfx_MOTracking.h"
#include "CascadedCoords.h"
#include <boost/weak_ptr.hpp>
#include <iomanip>

using namespace std;

boost::shared_ptr<const CCascadedCoords> CCascadedCoords::GetWorldAnchor() {
  static boost::shared_ptr<CCascadedCoords> pAnchor(new CCascadedCoordsAnchor());
  return pAnchor;
}

boost::shared_ptr<CCascadedCoords> CCascadedCoords::MakeCasCoords(double x, double y, double z, double yaw, double pitch, double roll, const std::string &rsName, boost::shared_ptr<const CCascadedCoords> pParent) {

  auto p = boost::shared_ptr<CCascadedCoords>(new CCascadedCoords(x,y,z,yaw,pitch,roll, pParent, rsName));
  pParent->_vpDescendants.push_back(boost::weak_ptr<ICascadedObjects>(p));
  return p;
}

boost::shared_ptr<CCascadedCoords> CCascadedCoords::MakeCasCoords(const CCoordinates3D &co, const std::string &rsName, boost::shared_ptr<const CCascadedCoords> pParent) {

  auto p = boost::shared_ptr<CCascadedCoords>(new CCascadedCoords(co, pParent, rsName));
  pParent->_vpDescendants.push_back(boost::weak_ptr<ICascadedObjects>(p));
  return p;
}

CCascadedCoords::CCascadedCoords(double x, double y, double z, double yaw, double pitch, double roll, boost::shared_ptr<const CCascadedCoords> pParent, const std::string &rsName) :  CCoordinates3D(0,0,0,0,0,0)
{
  _sName = rsName;
  _bChanged = true;
  _Local.SetPos(x,y,z);
  _Local.SetRPYAngle(yaw,pitch,roll);
  if (pParent) {
    _pParent = pParent;
    Update();
  }
}

CCascadedCoords::CCascadedCoords(const CCoordinates3D &co, boost::shared_ptr<const CCascadedCoords> pParent, const std::string &rsName) {

  _sName = rsName;
  _bChanged = true;
  _Local = co;
  if (pParent) {
    _pParent = pParent;
    Update();
  }
}

CCascadedCoords::~CCascadedCoords(void){}

void CCascadedCoords::AddChild(boost::shared_ptr<ICascadedObjects> pChild) const {
  _vpDescendants.push_back(boost::weak_ptr<ICascadedObjects>(pChild));
}

void CCascadedCoords::SetRot(const BoostMat& rot) {
  NotifyUpdate();
  _Local.SetRot(rot);
}

void CCascadedCoords::SetPos(double x, double y, double z) {
  NotifyUpdate();
  _Local.SetPos(x,y,z);
}

void CCascadedCoords::SetRPYAngle(double yaw, double pitch, double roll) {
  NotifyUpdate();
  _Local.SetRPYAngle(yaw, pitch, roll);
}

void CCascadedCoords::SetQuartenion(const boost::math::quaternion<double>& q) {
  NotifyUpdate();
  _Local.SetQuartenion(q);
}

void CCascadedCoords::Copy(const CCoordinates &rTarget) {
  NotifyUpdate();
  _Local.Copy(rTarget);
}

void CCascadedCoords::TransformLocal(const CCoordinates& coord) {
  throw std::logic_error("TransformLocal not implemented");
}

void CCascadedCoords::TransformWorld(const CCoordinates& coord) {
  throw std::logic_error("TransformWorld not implemented");
}

std::string CCascadedCoords::GetStr() const {
  Update();
  return CCoordinates3D::GetStr();
}


const BoostMat& CCascadedCoords::GetRot() const {
  Update();
  return CCoordinates3D::GetRot();
}

const BoostVec& CCascadedCoords::GetPos() const {
  Update();
  return CCoordinates3D::GetPos();
}

double CCascadedCoords::GetX() const {
  Update();
  return CCoordinates3D::GetX();
}

double CCascadedCoords::GetY() const {
  Update();
  return CCoordinates3D::GetY();
}

double CCascadedCoords::GetZ() const {
  Update();
  return CCoordinates3D::GetZ();
}

double CCascadedCoords::GetYaw() const {
  Update();
  return CCoordinates3D::GetYaw();
}

double CCascadedCoords::GetPitch() const {
  Update();
  return CCoordinates3D::GetPitch();
}

double CCascadedCoords::GetRoll() const {
  Update();
  return CCoordinates3D::GetRoll();
}

//子にupdateを伝播，その際消えている座標は取り除く
void CCascadedCoords::NotifyUpdate() const {

  _bChanged = true;
  auto it = _vpDescendants.begin();
  while (it != _vpDescendants.end()) {
    if (auto p = it->lock()) {
      p->NotifyUpdate();
      ++it;
    }
    else {
      it = _vpDescendants.erase(it);
    }
  }
}

void CCascadedCoords::UpdateImpl() {
  _bChanged = false;
  MakeGlobalCoord(*_pParent, _Local, *this);
}

void CCascadedCoords::Update() const {
  if (_bChanged) {
    _pParent->Update();
    CCascadedCoords *pThis = const_cast<CCascadedCoords*>(this);
    pThis->UpdateImpl();
  }
}