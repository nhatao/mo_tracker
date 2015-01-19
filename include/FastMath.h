#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include "fmath.hpp"


//テーブル化などで標準より高速化した関数群
namespace FastMath {

  inline double table_sin(double d);
  inline double table_cos(double d);
  inline double table_atan2(double y, double x);

  class CMathTables {
  public:
    static CMathTables &Get() {
      static CMathTables Instance;
      return Instance;
    }
    ~CMathTables() {
      delete[] _adSinTable;
      delete[] _adATanTable;
    }
    const double* GetAtanTable() const {return _adATanTable;}
    const double* GetSinTable() const {return _adSinTable;}

  protected:

    CMathTables() {
      BuildTable();
    }
    void BuildTable() {
      _adSinTable = new double[65536];
      _adATanTable = new double[65536];

      for (size_t i=0; i<65536; ++i) {
        _adSinTable[i] = sin(2*M_PI/65536*i);
        _adATanTable[i] = atan2(i/65536.0,1);
      }
    }
    double* _adSinTable;
    double* _adATanTable;
  };

  inline double table_sin(double d) {
    static const double dA = 65536/2/M_PI;
    return CMathTables::Get().GetSinTable()[(int)(d*dA) & 0x0000ffff];
  }
  inline double table_cos(double d) {
    static const double dA = 65536/2/M_PI;
    return CMathTables::Get().GetSinTable()[((int)(d*dA)+16384) & 0x0000ffff];
  }

  inline double table_atan2(double y, double x) {

    double dOffset;
    if (y>=0) {
      if (x>=0) {
        dOffset = 0;
      }
      else {
        double d = x;
        x = y;
        y = -d;
        dOffset = M_PI_2;
      }
    } 
    else {
      if (x<0) {
        x = -x;
        y = -y;
        dOffset = -M_PI;
      }
      else {
        double d = x;
        x = -y;
        y = d;
        dOffset = -M_PI_2;
      }
    }
    if (x>y) {
      return CMathTables::Get().GetAtanTable()[((int)(y/x*65536)) & 0x0000ffff] + dOffset;
    }
    else if (y>x) {
      return M_PI_2 - CMathTables::Get().GetAtanTable()[((int)(x/y*65536)) & 0x0000ffff] + dOffset;
    }
    else {
      return M_PI/4 + dOffset;
    }  
  }

#if 0
  //普通のhypotより4倍程度早い+オーバーフローがないことを保証
  //しかしsqrtより遅い
  inline double fast_hypot(double a, double b){
    a = fabs(a);
    b = fabs(b);
    if (a < b) std::swap(a, b);
    if (b == 0)  return a;
    const int ITERATION_NUMBER = 3;
    double s;  
    for (int i=0; i<ITERATION_NUMBER; i++){
      s=(b/a)*(b/a);
      s/=4+s;
      a+=2*a*s;
      b*=s;
    }
    return a;
  } 
#else
  inline double fast_hypot(const double& a, const double& b) {
    return sqrt(a*a+b*b);
  }

#endif

};
