#pragma once
#include "MatrixFuncs.h"
#include "Angle.h"
#include <cmath>

//距離と角度をデータとした2次元極座標系のベクトル
class CPolarVec
{
public:
  CPolarVec(void){
    _dDist = 0;
  }
  CPolarVec(double dDist, double dDeg) {
    _dDist = dDist;
    _Angle = dDeg;
  }
  virtual ~CPolarVec(void) {}
  double GetDist() const {return _dDist;}
  const CAngle& GetAngle() const {return _Angle;}
  void SetDist(double dDist) {_dDist = dDist;}
  void SetAngle(const CAngle& rAngle) {
    _Angle = rAngle;
  }

  CPolarVec(const BoostVec& rhs);
  CPolarVec& operator=(const BoostVec& rhs);

  BoostVec ToBoostVec() {
    BoostVec v(2);
    v(0) = _dDist * cos(_Angle);
    v(1) = _dDist * sin(_Angle);
    return v;
  }

private:
  double _dDist;
  CAngle _Angle;
};