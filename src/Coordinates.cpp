#include "StdAfx_MOTracking.h"

#include "MatrixFuncs.h"
#include "Coordinates.h"

#include <iostream>
#include <math.h>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <limits>
#include <stdexcept>
#include <string>

using namespace std;
using namespace boost::numeric;

CCoordinates::CCoordinates(int dim) : _dimension(dim)
{
  Init();
}

CCoordinates::~CCoordinates(void){}

void CCoordinates::Init() {

  _rot.resize(_dimension,_dimension);
  _pos.resize(_dimension);  

  BoostMat temp = ublas::identity_matrix<double>(_dimension);
  _rot = temp;
  _pos.clear();
}

void CCoordinates::Copy(const CCoordinates &rTarget) {

  if (rTarget.Dimension() != Dimension()) {
    ostringstream oss;
    oss << __FUNCTION__ << " Dimension Mismatch: Target=" << rTarget.Dimension() << " This=" << Dimension();
    throw std::logic_error(oss.str().c_str());
  }
  SetRot(rTarget.GetRot());
  SetPos(rTarget.GetPos());
}

void CCoordinates3D::Normalize() {
  double rpy[3];
  RPYAngle(_rot, rpy);
  RPYMatrix(rpy[0],rpy[1],rpy[2],_rot);
}

void CCoordinates2D::Normalize() {

  SetRotation(GetRotation());
}

void CCoordinates::TransformLocal(const CCoordinates& coord) {

  _pos = prod(_rot, coord._pos) + _pos;
  _rot = prod(_rot, coord._rot);

  Normalize();
}

void CCoordinates::MakeTransformLocal(CCoordinates &result, const CCoordinates& coord) const {

  result._pos = prod(_rot, coord._pos) + _pos;
  result._rot = prod(_rot, coord._rot);

  result.Normalize();
}


void CCoordinates::InverseTransformation(CCoordinates& rResultCo) const {
  rResultCo._rot = trans(_rot);
  rResultCo._pos = prod(rResultCo._rot, _pos);
  rResultCo._pos = rResultCo._pos*-1.0;
  rResultCo.Normalize();
}


void CCoordinates::TransformWorld(const CCoordinates& coord) {

  _pos = prod(coord._rot, _pos) + coord._pos;
  _rot = prod(coord._rot, _rot);

  Normalize();
}

// inv c2 inv 
// this->inv
// c1=result c2=coord
void CCoordinates::Transformation(CCoordinates &result, const CCoordinates& coord) const{

  result.Init();
  InverseTransformation(result);

  //transform-coords
  result._pos = result._pos + prod(result._rot, coord._pos);
  result._rot = prod(result._rot, coord._rot);
  result.Normalize();

}

void CCoordinates2D::SetRotation(double angle) {

  _rot(0,0) = cos(angle); _rot(0,1) = -sin(angle);
  _rot(1,0) = sin(angle); _rot(1,1) = cos(angle);
}

double CCoordinates2D::GetRotation() const {

  return atan2(_rot(1,0), _rot(0,0));
}


void CCoordinates2D::Rotate(double angle) {

  BoostMat mat(2,2);
  mat(0,0) = cos(angle); mat(0,1) = -sin(angle);
  mat(1,0) = sin(angle); mat(1,1) = cos(angle);
  _rot = prod(_rot, mat);
}

CCoordinates3D::CCoordinates3D() : CCoordinates(3)
{
}

CCoordinates3D::CCoordinates3D(double x, double y, double z, double yaw, double pitch, double roll) : CCoordinates(3)
{
  SetPos(x, y, z);
  SetRPYAngle(yaw, pitch, roll);
}


void CCoordinates3D::SetRPYAngle(double yaw, double pitch, double roll) {

  RPYMatrix(yaw, pitch, roll, _rot);
}

void CCoordinates3D::SetQuartenion(const boost::math::quaternion<double>& q) {


  using ::std::numeric_limits;
    
  double    a = q.R_component_1();
  double    b = q.R_component_2();
  double    c = q.R_component_3();
  double    d = q.R_component_4();
    
  double    aa = a*a;
  double    ab = a*b;
  double    ac = a*c;
  double    ad = a*d;
  double    bb = b*b;
  double    bc = b*c;
  double    bd = b*d;
  double    cc = c*c;
  double    cd = c*d;
  double    dd = d*d;
    
  double    norme_carre = aa+bb+cc+dd;
   
  if    (norme_carre <= numeric_limits<double>::epsilon())
  {
      ::std::string            error_reporting("Argument to quaternion_to_R3_rotation is too small!");
      ::std::underflow_error   bad_argument(error_reporting);
        
      throw(bad_argument);
  }
  _rot(0,0) = (aa+bb-cc-dd)/norme_carre;
  _rot(0,1) = 2*(-ad+bc)/norme_carre;
  _rot(0,2) = 2*(ac+bd)/norme_carre;
  _rot(1,0) = 2*(ad+bc)/norme_carre;
  _rot(1,1) = (aa-bb+cc-dd)/norme_carre;
  _rot(1,2) = 2*(-ab+cd)/norme_carre;
  _rot(2,0) = 2*(-ac+bd)/norme_carre;
  _rot(2,1) = 2*(ab+cd)/norme_carre;
  _rot(2,2) = (aa-bb-cc+dd)/norme_carre;
}

#include <iomanip>
#include <boost/io/ios_state.hpp>

std::ostream& operator<< (std::ostream &os, const CCoordinates &r) {
  boost::io::ios_flags_saver ifs(os);
  os << fixed;
  os << r._pos << " " << r._rot;
  return os;
}
std::istream& operator>> (std::istream &is, CCoordinates &r) {
  BoostVec vec; BoostMat rot;
  is >> vec >> rot;
  r.SetPos(vec); r.SetRot(rot);
  return is;
}

string CCoordinates2D::GetStr() const {

  ostringstream os;
  os << "co: " << _pos << "/" << "(" << Rad2Deg(GetRotation()) << ")" ;
  return os.str();

}
string CCoordinates3D::GetStr() const {
  ostringstream os;
  double rpy[3];
  RPYAngle(_rot, rpy, true);
  os << "co: " << _pos << "/" << "(" << Rad2Deg(rpy[0]) << "," << Rad2Deg(rpy[1]) << "," << Rad2Deg(rpy[2]) << ")" ;
  return os.str();
}

double CCoordinates3D::GetYaw() const {
  double rpy[3];
  RPYAngle(_rot, rpy);
  return rpy[0];
}
double CCoordinates3D::GetPitch() const {
  double rpy[3];
  RPYAngle(_rot, rpy);
  return rpy[1];
}
double CCoordinates3D::GetRoll() const {
  double rpy[3];
  RPYAngle(_rot, rpy);
  return rpy[2];
}

//rParent座標系のrChildをグローバル座標に変換
void MakeGlobalCoord(const CCoordinates3D &rParent, const CCoordinates3D &rChild, CCoordinates3D &rResult) {
//  rResult = rParent;
//  rResult.TransformLocal(rChild);
  rParent.MakeTransformLocal(rResult, rChild);
}

//グローバル座標系のrChildをrParent座標系に変換
void MakeLocalCoord(const CCoordinates3D &rParent, const CCoordinates3D &rChild, CCoordinates3D &rResult) {
  rParent.Transformation(rResult, rChild);
}


//rParent座標系のrChildをグローバル座標に変換
void MakeGlobalCoord(const CCoordinates2D &rParent, const CCoordinates2D &rChild, CCoordinates2D &rResult) {
//  rResult = rParent;
//  rResult.TransformLocal(rChild);
  rParent.MakeTransformLocal(rResult, rChild);
}

//グローバル座標系のrChildをrParent座標系に変換
void MakeLocalCoord(const CCoordinates2D &rParent, const CCoordinates2D &rChild, CCoordinates2D &rResult) {
  rParent.Transformation(rResult, rChild);
}
