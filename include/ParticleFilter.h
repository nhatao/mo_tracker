#pragma once

#include <iostream>
#include <vector>
#include "MatrixUtils.h"
#include "MatrixFuncs.h"
#include "SobolQuasiRandom.h"
#include <boost/random.hpp>
#include "StreamException.h"

//パーティクルフィルタの基礎クラス
//観測に用いるクラスを何でもいいようにテンプレート化。T=BoostVecでいいならCParticleFilterを用いる

template <class T>
class CParticleFilterImpl {

public:
  CParticleFilterImpl(size_t nParticleNum, size_t nStateDim, size_t nIterationNum=1, bool bUseQuasi=false);
  virtual ~CParticleFilterImpl();

  void GetCovarianceMatrix(BoostMat &rResult) const;
  virtual void ResetParticles(const BoostVec& rvInitState, const BoostVec& rvInitNoise);
  virtual const BoostVec& Update(const T &rvY);
  virtual const BoostVec& GetResult() const {return _vResult;}

  const BoostVec& GetSystemNoise() const {return _vSystemNoise;}
  void SetSystemNoise(const BoostVec& rvNoise) {
    if (rvNoise.size()==_nStateDim) _vSystemNoise = rvNoise;
    else {
      ESStreamException ess; ess << "SetSystemNoise size mismatch. Current:" << rvNoise.size() << " Correct: " << _nStateDim;
      throw ess;
    }
  }
  const BoostVec& GetMeasurementNoise() const {return _vMeasurementNoise;}
  void SetMeasurementNoise(const BoostVec& rvNoise) {
    if (rvNoise.size()==_nStateDim) _vMeasurementNoise = rvNoise;
    else {
      ESStreamException ess; ess << "SetMeasurementNoise size mismatch. Current:" << rvNoise.size() << " Correct: " << _nStateDim;
      throw ess;
    }
  }
  const std::vector<BoostVec> &GetParticles(bool bFiltered=false) const{
    if (bFiltered) return _avFilter;
    else return _avPredict;
  }

  size_t GetParticleNum() const {return _nParticleNum;}

  void SetProcessVector(const BoostVec &v) {
    if (v.size()==_nStateDim) _vProcessVector = v;
    else {
      ESStreamException ess; ess << "SetProcessVector size mismatch. Current:" << v.size() << " Correct: " << _nStateDim;
      throw ess;
    }
  }

protected:

  virtual void Predict();
  virtual void Weighting(const T &rvY);
  virtual void Resampling(const T &rvY);
  virtual void CalcResult();
  virtual double Likelihood(const BoostVec &rX, const T &rY) = 0;

  BoostVec MakeRandomVec(const BoostVec& rvState, const BoostVec& rvNoise) {
    BoostVec vResult = rvState;
    if (_bUseQuasi) {
      double aRandom[SOBOLMAXDIM];
      _qrand.GetRandom(_nStateDim, aRandom);
      for (size_t k=0; k<_nStateDim; ++k) vResult(k)+=((aRandom[k]-0.5)*2*rvNoise(k));
    }
    else {
      for (size_t k=0; k<_nStateDim; ++k) vResult(k)+=(Random()*rvNoise(k));
    }
    return vResult;
  }

  virtual void Process(BoostVec& rvX, const BoostVec& rvXBefore) {
    rvX = rvXBefore + _vProcessVector;
  }
  virtual void AddProcessNoise(BoostVec& rvX) {
    if (_bUseQuasi) {
      double aRandom[SOBOLMAXDIM];
      _qrand.GetRandom(_nStateDim, aRandom);
      for (size_t i=0; i<_nStateDim; ++i) {
        rvX(i)+=((aRandom[i]-0.5)*2*_vSystemNoise(i));
      }
    }
    else {
      for (size_t i=0; i<_nStateDim; ++i) {
        rvX(i)+=(Random()*_vSystemNoise(i));
      }
    }
  }

  size_t _nParticleNum;
  size_t _nIterationNum;
  size_t _nStateDim;
  std::vector<BoostVec> _avPredict;
  std::vector<BoostVec> _avFilter;

  BoostVec _vProcessVector;
  std::vector<double> _vWeight;
  BoostVec _vSystemNoise;
  BoostVec _vMeasurementNoise;
  BoostVec _vResult;
  CSobolQuasiRandom _qrand;
  boost::mt19937 _gen;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > Random;
  bool _bUseQuasi;
};

#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable: 4244)
#pragma warning (disable: 4267)
#endif 

#include <algorithm>
#include <boost/random.hpp>
#include <iostream>

template <class T>
CParticleFilterImpl<T>::CParticleFilterImpl(size_t nParticleNum, size_t nStateDim, size_t nIterationNum, bool bUseQuasi) 
: _gen(900000), Random(_gen, boost::normal_distribution<>(0.0,1.0)), _bUseQuasi(bUseQuasi) 
//: _gen(time(0)), Random(_gen, boost::normal_distribution<>(0.0,1.0)), _bUseQuasi(bUseQuasi) 
{
  _nParticleNum = nParticleNum;
  _nIterationNum = nIterationNum;
  _nStateDim = nStateDim;

  _avPredict.resize(_nParticleNum);
  for (size_t i=0; i<_nParticleNum; ++i) _avPredict[i].resize(_nStateDim);
  _avFilter.resize(_nParticleNum);
  for (size_t i=0; i<_nParticleNum; ++i) _avFilter[i].resize(_nStateDim);
  _vWeight.resize(_nParticleNum);
  _vResult.resize(_nStateDim);  
  _vProcessVector.resize(_nStateDim); std::fill(_vProcessVector.begin(), _vProcessVector.end(), 0);
  _vSystemNoise.resize(_nStateDim); std::fill(_vSystemNoise.begin(), _vSystemNoise.end(), 10);
  _vMeasurementNoise.resize(_nStateDim); std::fill(_vMeasurementNoise.begin(), _vMeasurementNoise.end(), 10);

  BoostVec vInit(_nStateDim); std::fill(vInit.begin(), vInit.end(), 0);
  BoostVec vInitN(_nStateDim); std::fill(vInitN.begin(), vInitN.end(), 1.0);
  ResetParticles(vInit,vInitN);

}

template <class T>
CParticleFilterImpl<T>::~CParticleFilterImpl() {

}

template <class T>
void CParticleFilterImpl<T>::GetCovarianceMatrix(BoostMat &rResult) const{

}

template <class T>
void CParticleFilterImpl<T>::ResetParticles(const BoostVec& rvInitState, const BoostVec& rvInitNoise) {

  if (rvInitState.size() != _nStateDim) {
    ESStreamException oss; oss << "CParticleFilterImpl::ResetParticles dimension mismatch. required=" << _nStateDim << " rInitState:" << rvInitState.size();
    throw oss;
  }
  std::vector<BoostVec>::iterator itFilter = _avFilter.begin();
  for(;itFilter!=_avFilter.end();++itFilter) {
    (*itFilter) = MakeRandomVec(rvInitState, rvInitNoise);
    _vResult += (*itFilter);
  } 
  _avPredict = _avFilter;
  std::fill(_vWeight.begin(), _vWeight.end(), 1.0/_vWeight.size());
  _vResult/=_avFilter.size();
}


template <class T>
const BoostVec& CParticleFilterImpl<T>::Update(const T &rvY) {

  for (size_t i=0; i<_nIterationNum; ++i) {
    Predict();
    Weighting(rvY);
    Resampling(rvY);
  }
  CalcResult();
  return _vResult;
}

template <class T>
void CParticleFilterImpl<T>::Predict() {

  std::vector<BoostVec>::iterator itPredict = _avPredict.begin();
  std::vector<BoostVec>::const_iterator itFilter = _avFilter.begin();
  for(;itPredict!=_avPredict.end();++itPredict,++itFilter) {
    Process(*itPredict, *itFilter);
    AddProcessNoise(*itPredict);
  }
}

template <class T>
void CParticleFilterImpl<T>::Weighting(const T &rvY) {

  std::vector<BoostVec>::iterator itPredict = _avPredict.begin();
  std::vector<double>::iterator itWeight = _vWeight.begin();  
  double dWeightSum = 0;
  for(;itPredict!=_avPredict.end();++itPredict,++itWeight) {
    *itWeight = Likelihood(*itPredict, rvY);
    dWeightSum += (*itWeight);
  }
  //正規化
  for(itWeight=_vWeight.begin();itWeight!=_vWeight.end();++itWeight) (*itWeight)/=dWeightSum;
}

template <class T>
void CParticleFilterImpl<T>::Resampling(const T &rvY) {
  size_t l=0, j=0;
  double c1=0;
  while (l<_nParticleNum) {
    c1 += _vWeight[l];
    double u = (j+0.5)/_nParticleNum;
    while ( (j<_nParticleNum) && (u<=c1) ) {
      _avFilter[j] = _avPredict[l];
      ++j;
      u = (j+0.5)/_nParticleNum;
    }
    ++l;
  }
}

template <class T>
void CParticleFilterImpl<T>::CalcResult() {

  std::fill(_vResult.begin(), _vResult.end(), 0);
  std::vector<BoostVec>::const_iterator itPredict = _avPredict.begin();
  std::vector<double>::const_iterator itWeight = _vWeight.begin();  
  for(;itPredict!=_avPredict.end();++itPredict,++itWeight) {
    _vResult+=(*itWeight)*(*itPredict);
  }
}

//パーティクルフィルタの基礎クラス
//x(t+1) = x(t) + u
//y(t) = x(t) + w
//の基本モデルに対応。u,wは共分散0
//モデルを変えるには、Likelihood,Process,AddProcessNoiseを変更すればよい

class CParticleFilter : public CParticleFilterImpl<BoostVec> {
public:

  CParticleFilter(size_t nParticleNum, size_t nStateDim, size_t nIterationNum=1) 
    : CParticleFilterImpl<BoostVec>(nParticleNum,nStateDim,nIterationNum) {};
  virtual ~CParticleFilter() {};

protected:

  virtual double Likelihood(const BoostVec &rX, const BoostVec &rY) {
    using namespace boost::numeric::ublas;
    double xSx = 0;
    for (size_t i=0; i<_nStateDim; ++i)
      xSx += pow((rX(i)-rY(i))/_vMeasurementNoise(i), 2);
    return exp(-0.5*xSx); //確率密度関数の係数部分(1/(sqrt(2*pi)*sigma))はいらない そのあとの正規化で消えるため
  }
};


#ifdef _MSC_VER
#pragma warning (pop)
#endif 
