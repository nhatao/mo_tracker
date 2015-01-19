#pragma once
#include <boost/operators.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

/*
  角度を表すクラス
  degree/radian 変換、一周チェック、doubleへの変換などが可能 
  get()の範囲は -PI以上PI未満 (atan2と同じ)、get_positive()の範囲は 0以上2PI未満
   bool operator<(const CAngle& rhs) : 左回りの方が近ければtrue, そうでなければfalse 
   ちょうどM_PI[rad]ならtrue
     if (CAngle(Deg2Rad(30)) < CAngle(Deg2Rad(60))) -> true;
     if (CAngle(Deg2Rad(30)) < CAngle(Deg2Rad(330))) -> false;
*/

const double ONEDEG = M_PI/180.0;
inline double Rad2Deg(double rad) {return rad/ONEDEG;};
inline double Deg2Rad(double deg) {return deg*ONEDEG;};

//class CAngle : private boost::operators<CAngle> //これを使うとIntellisenseが効かなくなる
class CAngle 
{
public:

  CAngle(void) { _dVal = 0;}

  CAngle(double d, bool bRadian=true) { 
    if (!bRadian) SetVal(Deg2Rad(d));
    else SetVal(d);
  }
  virtual ~CAngle(void){}
  double get() const {return _dVal;}
  double get_deg() const {return Rad2Deg(_dVal);}
  double get_positive() const {if (_dVal<0) return _dVal+2*M_PI; else return _dVal;} 
  double get_deg_positive() const {if (_dVal<0) return Rad2Deg(_dVal)+360; else return Rad2Deg(_dVal);} 

  void set(double d) {SetVal(d);}
  void set_deg(double d) {SetVal(Deg2Rad(d));}

  bool operator<(const CAngle& rhs) const { 
    if (rhs._dVal<_dVal) {
      return (rhs._dVal+2*M_PI-_dVal)<=M_PI;
    }
    else return (rhs._dVal-_dVal)<=M_PI;
  }
  bool operator==(const CAngle& rhs) const { return _dVal == rhs._dVal; }
  CAngle& operator+=(const CAngle& rhs) { SetVal(_dVal+rhs._dVal); return *this; }
  CAngle& operator-=(const CAngle& rhs) { SetVal(_dVal-rhs._dVal); return *this; }
  CAngle& operator*=(const CAngle& rhs) { SetVal(_dVal*rhs._dVal); return *this; }
  CAngle& operator/=(const CAngle& rhs) { SetVal(_dVal/rhs._dVal); return *this; }

  bool operator!=(const CAngle& rhs) const { return !(operator==(rhs)); }
  bool operator> (const CAngle& rhs) const { return !((operator==(rhs))||(operator<(rhs))); }
  bool operator<=(const CAngle& rhs) const { return (operator<(rhs))||(operator==(rhs)); }
  bool operator>=(const CAngle& rhs) const { return (operator>(rhs))||(operator==(rhs)); }

  CAngle operator+(const CAngle& rhs) const { CAngle res = *this; res.operator+=(rhs); return res;}
  CAngle operator-(const CAngle& rhs) const { CAngle res = *this; res.operator-=(rhs); return res;}
  CAngle operator*(const CAngle& rhs) const { CAngle res = *this; res.operator*=(rhs); return res;}
  CAngle operator/(const CAngle& rhs) const { CAngle res = *this; res.operator/=(rhs); return res;}

  template<typename T>
  CAngle operator*(const T& rhs) {CAngle res = *this; res.operator*=(rhs); return res;}
  template<typename T>
  CAngle operator/(const T& rhs) {CAngle res = *this; res.operator/=(rhs); return res;}

  //rA1->this->rA2のならびになっているかどうか
  bool is_sandwiched(const CAngle &rA1, const CAngle &rA2) const{
    CAngle a1 = rA2-rA1;
    CAngle a2 = (*this)-rA1;
    return (a1.get_positive()>=a2.get_positive());
  }

  CAngle& operator=(const CAngle& rhs){ SetVal(rhs._dVal); return *this;}
  CAngle& operator=(const double& d)  { SetVal(d); return *this;}

protected:

  void SetVal(double d) {
    if ((d < M_PI) && (d >= -M_PI)) {
      _dVal = d;
    }
    else {
      _dVal = fmod(d, 2*M_PI);
      if (_dVal >= M_PI) _dVal-=2*M_PI;
      else if (_dVal < -M_PI) _dVal+=2*M_PI;
    }
  }

  double _dVal;
};

inline std::ostream& operator<< (std::ostream &os, const CAngle& r) {
  os << r.get();
  return os;
}
inline std::istream& operator>> (std::istream &is, CAngle& r) {
  double d;
  is >> d;
  r.set(d);
  return is;
}

inline double cos(const CAngle &r) {return std::cos(r.get());}
inline double sin(const CAngle &r) {return std::sin(r.get());}
inline double tan(const CAngle &r) {return std::tan(r.get());}

