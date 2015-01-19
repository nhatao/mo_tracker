#pragma once
#include "MatrixFuncs.h"
#include <math.h>
#include <iostream>

//ax+by+c=0; sqrt(a*a+b*b)=1 の状態を保存
class CLine
{
public:
  CLine() {
    SetParam(1,0,0);
  }
  CLine(double a, double b, double c) {
    SetParam(a,b,c);
  }
  CLine(double x1, double y1, double x2, double y2) {
    SetPoints(x1,y1,x2,y2);
  }
  virtual ~CLine() {}

  void SetParam(double a, double b, double c) {
    double len = sqrt(a*a+b*b);
    if (a<0) len = -len;
    _a = a/len;
    _b = b/len;
    _c = c/len;
  }
  virtual void SetPoints(double x1, double y1, double x2, double y2) {
    SetParam(y1-y2, x2-x1, (x1-x2)*y1-(y1-y2)*x1);  
  }
  double GetGrad() const {
    if (_b==0) return M_PI/2;
    else return atan(-_a/_b);
  }
  double CalcY(double x) const{
    if (_b==0) return (-_c/_a);
    else return (_a*x+_c)/(-_b);
  }
  double CalcX(double y) const{
    if (_a==0) return (-_c/_b);
    else return (_b*y+_c)/(-_a);
  }
  double GetA() const {return _a;}
  double GetB() const {return _b;}
  double GetC() const {return _c;}

  virtual double GetDistanceFromPoint(double x, double y) const {
    return fabs(_a*x + _b*y + _c);
  }
  virtual double GetDistanceFromPoint(const BoostVec& BoostVec) const {
    return GetDistanceFromPoint(BoostVec(0), BoostVec(1));
  }
  virtual double GetDistanceFromOrigin() const {return fabs(_c);}

  bool IsUpper(double x, double y, bool bIncludeLine = true) const{ //直線より上にあればtrue, 境界線上を含まない
    //垂直なら左側がtrue <-これでいいのか？
    if (bIncludeLine) return (CalcY(x)<=y);
    else return (CalcY(x)<y);
  }
  bool IsUpper(const BoostVec& BoostVec, bool bIncludeLine = true) const {
    return IsUpper(BoostVec(0), BoostVec(1), bIncludeLine);
  }
  bool IsOnLine(double x, double y) const { //直線上にあればtrue
    return (CalcY(x)==y);
  }
  bool IsOnLine(const BoostVec& BoostVec) const {
    return IsOnLine(BoostVec(0), BoostVec(1));
  }
  virtual double GetSignedDistanceFromOrigin() const {return _c;}

  virtual bool GetIntersection(const CLine &rTarget, BoostVec &rResult) const {
    if ( (_a==rTarget.GetA()) && (_b==rTarget.GetB()) ) return false;
    rResult.resize(2);
    if (rTarget.GetB() == 0) {
      rResult(0) = -rTarget.GetC()/rTarget.GetA();
      rResult(1) = (-rResult(0)*_a-_c)/_b;
      return true;
    }
    if (_b == 0) {
      rResult(0) = -_c/_a;
      rResult(1) = (-rResult(0)*rTarget.GetA()-rTarget.GetC())/rTarget.GetB();
      return true;
    }
    double a1 = _a/_b;
    double a2 = rTarget.GetA()/rTarget.GetB();
    double c1 = _c/_b;
    double c2 = rTarget.GetC()/rTarget.GetB();
    double x = (c2-c1)/(a1-a2);
    double y = -(_a*x+_c)/_b;
    rResult(0) = x; rResult(1) = y;
    return true;
  }
  virtual double GetNearestPoint(const BoostVec &rTarget, BoostVec &rResult) const {
    rResult(0) = _b*_b*rTarget(0)-_a*_b*rTarget(1)-_a*_c;
    rResult(1) = CalcY(rResult(0));
    return GetDistanceFromPoint(rTarget);
  }
  template <class T>
  bool EstimateLine(T itBegin, T itEnd) {
    if(itBegin==itEnd) return false;

    double xvar=0, yvar=0, checknum=0, xx, yy;
    double xmean=0, ymean=0;
    T it = itBegin;
    int num = 0;
    while (it!=itEnd) {
      xmean+=(*it)(0);
      ymean+=(*it)(1);
      ++it;
      ++num;
    }
    xmean/=num;
    ymean/=num;

    it = itBegin;
    while (it!=itEnd) {
      xx = xmean - (*it)(0);
      yy = ymean - (*it)(1);
      xvar += xx*xx;
      yvar += yy*yy;
      checknum += xx*yy;
      ++it;
    }
    xvar = sqrt(xvar / num);
    yvar = sqrt(yvar / num);
    if (checknum < 0) xvar = -xvar;
    SetParam(yvar, -xvar, xvar*ymean-yvar*xmean);
    return true;    
  };
protected:
  double _a;
  double _b;
  double _c;
};

class CLineWithEndPoint : public CLine {
public:

  CLineWithEndPoint() : CLine() {
    _BoostVecBegin.resize(2);
    _BoostVecEnd.resize(2);
  };
  CLineWithEndPoint(const BoostVec& BoostVecBegin, const BoostVec& BoostVecEnd) : CLine() {
    _BoostVecBegin.resize(2);
    _BoostVecEnd.resize(2);
    SetBoostVec(BoostVecBegin, BoostVecEnd);
  }
  CLineWithEndPoint(double x1, double y1, double x2, double y2) : CLine() {
    _BoostVecBegin.resize(2);
    _BoostVecEnd.resize(2);
    SetPoints(x1, y1, x2, y2);
  }

  virtual void SetPoints(double x1, double y1, double x2, double y2) {
    _BoostVecBegin(0) = x1; _BoostVecBegin(1) = y1;
    _BoostVecEnd(0) = x2; _BoostVecEnd(1) = y2;
    SetParam(y1-y2, x2-x1, (x1-x2)*y1-(y1-y2)*x1);
  }
  void SetBoostVec(const BoostVec& BoostVecBegin, const BoostVec& BoostVecEnd) {
//    _BoostVecBegin=BoostVecBegin; _BoostVecEnd=BoostVecEnd;
    double x1 = BoostVecBegin(0);
    double y1 = BoostVecBegin(1);
    double x2 = BoostVecEnd(0);
    double y2 = BoostVecEnd(1);
    _BoostVecBegin(0) = x1; _BoostVecBegin(1) = y1;
    _BoostVecEnd(0) = x2; _BoostVecEnd(1) = y2;
    SetParam(y1-y2, x2-x1, (x1-x2)*y1-(y1-y2)*x1);
  }
  virtual ~CLineWithEndPoint() {}

  virtual double GetDistanceFromPoint(const BoostVec& BoostVec) const {
    using namespace std;
    double d1 = fabs(_a*BoostVec(0) + _b*BoostVec(1) + _c);
    double d2 = Distance2D(BoostVec,_BoostVecBegin);
    double d3 = Distance2D(BoostVec,_BoostVecEnd);
    double dd = Distance2D(_BoostVecBegin,_BoostVecEnd);
    double d = max(d2,d3);
    if (d-d1*d1 <= dd) {//内部
      return d1;
    }
    else {//外部
      return sqrt(min(d2,d3));
    }
  }
  virtual double GetDistanceFromPoint(double x, double y) const {
    BoostVec v(2); v(0) = x; v(1) = y;
    return GetDistanceFromPoint(v);
  }
  virtual double GetNearestPoint(const BoostVec &rTarget, BoostVec &rResult) const {

    using namespace std;
    double d1 = fabs(_a*rTarget(0) + _b*rTarget(1) + _c);
    double d2 = Distance2D(rTarget,_BoostVecBegin);
    double d3 = Distance2D(rTarget,_BoostVecEnd);
    double dd = Distance2D(_BoostVecBegin,_BoostVecEnd);
    double d = max(d2,d3);
    if (d-d1*d1 <= dd) {//内部
      CLine::GetNearestPoint(rTarget,rResult);
      return d1;
    }
    else {//外部
      if (d2>d3) { rResult=_BoostVecEnd; }
      else {rResult = _BoostVecBegin;}
      return sqrt(min(d2,d3));
    }
  }
  const BoostVec& GetBegin() {return _BoostVecBegin;}
  const BoostVec& GetEnd()   {return _BoostVecEnd;}

  virtual double GetDistanceFromOrigin() const {return GetDistanceFromPoint(0,0);}

  virtual bool GetIntersection(const CLine &rTarget, BoostVec &rResult) const {

    const CLineWithEndPoint *pTarget = dynamic_cast<const CLineWithEndPoint *>(&rTarget);
    if (!pTarget) {
      return CLine::GetIntersection(rTarget, rResult);
    }

    if ( (_a==rTarget.GetA()) && (_b==rTarget.GetB()) ) return false;
    rResult.resize(2);

    BoostVec v =    pTarget->_BoostVecBegin - this->_BoostVecBegin;
    BoostVec seg1 = this->_BoostVecEnd      - this->_BoostVecBegin;
    BoostVec seg2 = pTarget->_BoostVecEnd   - pTarget->_BoostVecBegin;

    double Crs_v_v1 = v(0)*seg1(1) - v(1)*seg1(0);
    double Crs_v_v2 = v(0)*seg2(1) - v(1)*seg2(0);

    double Crs_v1_v2 = seg1(0)*seg2(1) - seg1(1)*seg2(0);
    double t1 = Crs_v_v2 / Crs_v1_v2;
    double t2 = Crs_v_v1 / Crs_v1_v2;

//    double outT1 = Crs_v_v2 / Crs_v1_v2;
//    double outT2 = Crs_v_v1 / Crs_v1_v2;

    const float eps = 0.00001f;
    if ( t1 + eps < 0 || t1 - eps > 1 || t2 + eps < 0 || t2 - eps > 1 ) {
      // 交差していない
      return false;
    }

    rResult = this->_BoostVecBegin + (this->_BoostVecEnd - this->_BoostVecBegin) * t1;
    return true;
  }

protected:
  
  BoostVec _BoostVecBegin;
  BoostVec _BoostVecEnd;
};
