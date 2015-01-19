#pragma once
#define _USE_MATH_DEFINES
#include <float.h>
#include <math.h>
#include "MatrixUtils.h"
#include <boost/math/quaternion.hpp>
#include <boost/numeric/ublas/lu.hpp>

//RPY角度から回転行列に変換
//void RPYMatrix (double yaw, double pitch, double roll, BoostMat &mDist);
//回転行列からRPYに変換
//void RPYAngle (const BoostMat &mRPYMatrix, double *pResult, bool isCosPitchPlus = true); 
inline void RPYMatrix(double yaw, double pitch, double roll, BoostMat &mDist) {
    
  mDist.resize(3,3);
  double cy = cos(yaw); double sy = sin(yaw);
  double cp = cos(pitch); double sp = sin(pitch); 
  double cr = cos(roll); double sr = sin(roll);

  mDist(0,0) = cy*cp;
  mDist(1,0) = sy*cp;
  mDist(2,0) = -sp;
  mDist(0,1) = cy*sp*sr - sy*cr;
  mDist(1,1) = sy*sp*sr + cy*cr;
  mDist(2,1) = cp*sr;
  mDist(0,2) = cy*sp*cr + sy*sr;
  mDist(1,2) = sy*sp*cr - cy*sr;
  mDist(2,2) = cp*cr;
}

inline bool eps_eq(double a, double b, double eps = 10*DBL_MIN) {
  if (fabs(a-b) < eps) return true;
  else return false;
}


inline void RPYAngle (const BoostMat &mRPYMatrix, double *pResult, bool isCosPitchPlus=true) {

  double a = atan2(mRPYMatrix(1,0), mRPYMatrix(0,0));
  if (!isCosPitchPlus) a+=M_PI;
  if (a>M_PI) a = a - 2*M_PI;
  pResult[0] = a;
  double sa = sin(a); double ca = cos(a);
  pResult[1] = atan2(-mRPYMatrix(2,0), ca*mRPYMatrix(0,0)+sa*mRPYMatrix(1,0));
  pResult[2] = atan2(sa*mRPYMatrix(0,2)-ca*mRPYMatrix(1,2), 
                  -sa*mRPYMatrix(0,1)+ca*mRPYMatrix(1,1));
}

inline double GetTr(const BoostMat& m) {
  size_t size = m.size1();
  double result = 0;
  for(size_t i=0; i<size; i++) {
    result += m(i,i);
  }
  return result;
}

//Distanceと書いてあるけどsqrtしてない
template <class T1, class T2> 
double Distance2D(const T1& a, const T2& b) {
  return (a(0)-b(0))*(a(0)-b(0)) + (a(1)-b(1))*(a(1)-b(1));
}
template <class T>
inline double DistanceFromOrigin2D(const T& a) {
  return (a(0)*a(0) + a(1)*a(1));
}
template <class T>
inline double Get2DDeg(const T &vec) {
  return Rad2Deg(atan2(vec(1), vec(0)));
}

#ifdef _MSC_VER
inline double round(double num) {
  return floor(num+0.5);
}
#else 
#define _hypot hypot
#endif
inline int roundi(double num) {
  return (int)floor(num+0.5);
}

//not sqrt
template <class T>
double vDistance2(const T& a, const T& b) {
  double r = 0;
  for(size_t i=0; i<a.size(); ++i) {
    r+=(a(i)-b(i))*(a(i)-b(i));
  }
  return r;
}

bool InverseMatrix(const BoostMat& rTarget, BoostMat& rResult);

double Determinant2x2(const BoostMat& rMat);
double Determinant3x3(const BoostMat& rMat);

double Determinant(const BoostMat& m);


template <typename T>
double Covariance(const T& v1, const T& v2, double dm1 = DBL_MAX, double dm2 = DBL_MAX) {

  size_t nSize = v1.size();
  typename T::const_iterator it1, it2;
  if (dm1 == DBL_MAX) {
    dm1 = 0;
    for (it1 = v1.begin(); it1 != v1.end(); ++it1) {
      dm1 += (*it1);
    }   
    dm1/=nSize;
  }
  if (dm2 == DBL_MAX) {
    dm2 = 0;
    for (it1 = v2.begin(); it1 != v2.end(); ++it1) {
      dm2 += (*it1);
    }   
    dm2/=nSize;
  }
  double d = 0;
  for (it1=v1.begin(), it2=v2.begin(); it1!=v1.end(); ++it1, ++it2) {
    d += ((*it1)-dm1)*((*it2)-dm2);
  }
  return d/nSize;
}


template <class T>
BoostMat MakeCovarianceMatrix(T rvFrom, T rvTo, BoostVec &vMean) {
  using namespace std;

  size_t nMatSize = rvFrom->size();
  size_t nVecNum = rvTo-rvFrom;
  BoostMat mResult = boost::numeric::ublas::zero_matrix<double>(nMatSize,nMatSize);
  vMean = boost::numeric::ublas::zero_vector<double>(nMatSize);
  T it1 = rvFrom;
  for (; it1 != rvTo; ++it1) {
    for (size_t i=0; i<nMatSize; ++i) {
      vMean(i) += (*it1)(i) / nVecNum;
    }
  }
  it1 = rvFrom;
  BoostVec v1;
  for (; it1 != rvTo; ++it1) {
    v1 = (*it1)-vMean;
    for (size_t i=0; i<nMatSize; ++i) {
      mResult(i,i) += v1(i)*v1(i)/nVecNum;
      for (size_t j=i+1; j<nMatSize; ++j) {
        double d = v1(i)*v1(j)/nVecNum;
        mResult(i,j) += d;
        mResult(j,i) += d;
      }
    }
  }
  return mResult;
}

template <class T>
BoostMat MakeCovarianceMatrix(T rvFrom, T rvTo) {
  BoostVec vMean;
  return MakeCovarianceMatrix(rvFrom, rvTo, vMean);
}


//BoostVecのvectorから共分散行列を作成
template <class T>
BoostMat MakeCovarianceMatrix(const std::vector<T>& rv) {
  return MakeCovarianceMatrix(rv.begin(), rv.end());
}

/*
#include "Lapack.h"
//特異値分解 特異値をmatrixで返す
template <class M>
bool GetSVD(const M& A, M& U, M& S, M& V) {

  namespace ublas = boost::numeric::ublas;
  ublas::vector<double> s((std::min)(A.size1(), A.size2()));
  ublas::matrix<double> CU(A.size1(), A.size1()), CVT(A.size2(), A.size2());

  int n = dgesvd(A, CU.data().begin(), s.data().begin(), CVT.data().begin());
  if (n==0) {
    U = trans(CU);
    V = trans(CVT);
    S = ublas::zero_matrix<double>(A.size1(), A.size2());
    for (std::size_t i = 0; i < s.size(); ++i) {
      S(i, i) = s[i];
    }
    return true;
  }
  else {
    return false;
  }

}

//特異値分解 特異値をvectorで返す
template <class M, class T>
bool GetSVD(const M& A, M& U, T& S, M& V) {

  namespace ublas = boost::numeric::ublas;
  ublas::vector<double> s((std::min)(A.size1(), A.size2()));
  ublas::matrix<double> CU(A.size1(), A.size1()), CVT(A.size2(), A.size2());
  S.resize((std::min)(A.size1(), A.size2()));

  int n = dgesvd(A, CU.data().begin(), S.data().begin(), CVT.data().begin());
  if (n==0) {
    U = trans(CU);
    V = trans(CVT);
    return true;
  }
  else {
    return false;
  }

}

template <class M>
bool PseudoInverse(const M& rTarget, M& rResult) {

  M a;
  if (InverseMatrix(prod (trans(rTarget), rTarget), a)) {
    rResult = prod(a, trans(rTarget));
    return true;
  }
  else {
    return false;
  }
}

template <class M>
bool PseudoInverse2(const M& rTarget, M& rResult) {

  M V, U, S;
  if (GetSVD(rTarget, U, S, V)) {
    for (size_t i=0; i<std::min(S.size1(), S.size2()); ++i) {
      S(i,i)=1.0/S(i,i);
    }
    rResult = prod( (BoostMat)prod(trans(V), S), trans(U));
    return true;
  }
  return false;

}
*/


