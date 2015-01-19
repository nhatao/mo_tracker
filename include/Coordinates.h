#pragma once

#include "MatrixFuncs.h"
#include "MatrixUtils.h"
#include <ostream>
#include <boost/math/quaternion.hpp>

class CBothVec;
class CPolarVec;

class CCoordinates
{
public:

  virtual ~CCoordinates();
  void Init();

  virtual void SetRot(const BoostMat& rot) {_rot = rot;}
  virtual void SetPos(const BoostVec& pos) {_pos = pos;}

  virtual const BoostMat& GetRot() const {return _rot;}
  virtual const BoostVec& GetPos() const {return _pos;}
  int Dimension() const {return _dimension;};

  virtual void Copy(const CCoordinates &rTarget);
  //この座標系で表現されるvec をワールド座標系の表現に変換する
  template <class T1, class T2>
  void TransformVector (const T1& vec, T2& result) const  { 
    result = prod(_rot,vec) + _pos;
  }
  //この座標系で表現されるvec をワールド座標系の表現に変換する
  template <class T1>
  void TransformVector (T1& vec) const {
    vec = prod(_rot,vec) + _pos;
  }
  //ワールド座標系におけるvec をローカル座標系の表現に逆変換する
  template <class T1, class T2>
  void InverseTransformVector (const T1& vec, T2& result) const { 
    result = prod(trans(_rot), vec) - prod(trans(_rot), _pos);
  }
  //ワールド座標系におけるvec をローカル座標系の表現に逆変換する
  template <class T1>
  void InverseTransformVector (T1& vec) const  {
    vec = prod(trans(_rot), vec) - prod(trans(_rot), _pos);
  }
  template <class T1, class T2>
  void VectorLocalToGlobal (const T1& vec, T2& result) const  { 
    TransformVector(vec, result);
  }
  template <class T1>
  void VectorLocalToGlobal (T1& vec) const {
    TransformVector(vec);
  }
  template <class T1, class T2>
  void VectorGlobalToLocal (const T1& vec, T2& result) const { 
    InverseTransformVector(vec, result);
  }
  template <class T1>
  void VectorGlobalToLocal (T1& vec) const  {
    InverseTransformVector(vec);
  }
  virtual void TransformLocal(const CCoordinates& coord); //coordはthis座標系、thisはworld座標系としたとき、thisをworld座標系でのcoordに変換 
  virtual void TransformWorld(const CCoordinates& coord); //coordはworld座標系,thisはcoord座標系としたとき、thisをworld座標系に直す 
  virtual void Transformation(CCoordinates &result, const CCoordinates& coord) const; //両方ともworld座標系、this座標系でのcoord座標系を返す 
  virtual void MakeTransformLocal(CCoordinates &result, const CCoordinates& coord) const;//coordはthis座標系、thisはworld座標系としたとき、world座標系でのcoord座標系を返す
  virtual void InverseTransformation(CCoordinates& rResultCo) const;

  friend std::ostream& operator<< (std::ostream &os, const CCoordinates &r);
  friend std::istream& operator>> (std::istream &is, CCoordinates &r);
  virtual void Normalize() = 0;
  virtual std::string GetStr() const = 0;

protected:
  CCoordinates(int dim);

  BoostMat _rot;
  BoostVec _pos;
  const int _dimension;
};

class CCoordinates2D : public CCoordinates {

public:
  CCoordinates2D() : CCoordinates(2) {};
  CCoordinates2D(double x, double y, double theta) : CCoordinates(2){
    SetPos(x,y);
    SetRotation(theta);
  }
  ~CCoordinates2D(){};
  void SetRotation(double angle);//指定した角度から回転行列を作る
  double GetRotation() const; //現在の回転行列から回転角を得る
  void Rotate(double angle);//座標系を指定した角度だけ回転
  void SetPos(double x, double y) {_pos(0) = x; _pos(1) = y;}
  virtual void Normalize();
  virtual std::string GetStr() const;

  CCoordinates2D& operator= (const CCoordinates2D &rhs) {
    if(this == &rhs) return *this;   //自己代入
    _rot = rhs._rot;
    _pos = rhs._pos;
    return *this;
  }

  //matrix作成時の誤差などによってうまくいかないことがある eps_eqを使う方がよい
  bool operator==(const CCoordinates2D& rhs) const{
    return ( (_pos(0) == rhs._pos(0)) && (_pos(1) == rhs._pos(1)) 
      && (GetRotation() == rhs.GetRotation()) );
  }
  bool operator!=(const CCoordinates2D& rhs) const{
    return !this->operator==(rhs);
  }

};


inline bool eps_eq (const CCoordinates2D& c2, const CCoordinates2D& c1) 
{
  return ( (c1.GetPos()(0) == c2.GetPos()(0)) && (c1.GetPos()(1) == c2.GetPos()(1))
    && eps_eq(c1.GetRotation(), c2.GetRotation(), 1.0e-10));
}

class CCoordinates3D : public CCoordinates {

public:
  CCoordinates3D();
  CCoordinates3D(const CCoordinates3D &rhs)  : CCoordinates(3) {
    (*this) = rhs;
  }
  CCoordinates3D(const CCoordinates2D &rhs)  : CCoordinates(3) {
    (*this) = rhs;
  }

  CCoordinates3D(double x, double y, double z, double yaw, double pitch, double roll);
  ~CCoordinates3D(){};

  virtual void SetRPYAngle(double yaw, double pitch=0, double roll=0); //ラジアンで指定
  virtual void SetQuartenion(const boost::math::quaternion<double>& q);
  virtual void SetPos(double x, double y, double z) {_pos(0) = x; _pos(1) = y; _pos(2) = z;}
  void Normalize();
  virtual std::string GetStr() const;

  virtual double GetX() const {return GetPos()(0);}
  virtual double GetY() const {return GetPos()(1);}
  virtual double GetZ() const {return GetPos()(2);}
  virtual double GetYaw() const;
  virtual double GetPitch() const;
  virtual double GetRoll() const;

  CCoordinates3D& operator= (const CCoordinates3D &rhs) {
    if(this == &rhs) return *this;   //自己代入
    _rot = rhs._rot;
    _pos = rhs._pos;
    return *this;
  }
  CCoordinates3D& operator= (const CCoordinates2D &rhs) {
    SetPos(rhs.GetPos()(0), rhs.GetPos()(1), 0);
    SetRPYAngle(rhs.GetRotation(), 0, 0);
    return *this;
  }
};

std::ostream& operator<< (std::ostream &os, const CCoordinates &r);
std::istream& operator>> (std::istream &is, CCoordinates &r);

//rParent座標系のrChildをグローバル座標に変換
void MakeGlobalCoord(const CCoordinates3D &rParent, const CCoordinates3D &rChild, CCoordinates3D &rResult);
void MakeGlobalCoord(const CCoordinates2D &rParent, const CCoordinates2D &rChild, CCoordinates2D &rResult);
//グローバル座標系のrChildをrParent座標系に変換
void MakeLocalCoord(const CCoordinates3D &rParent, const CCoordinates3D &rChild, CCoordinates3D &rResult);
void MakeLocalCoord(const CCoordinates2D &rParent, const CCoordinates2D &rChild, CCoordinates2D &rResult);
