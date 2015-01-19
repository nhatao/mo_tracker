#include "StdAfx_MOTracking.h"
#include "JPDAFilter.h"
#include "FastMath.h"
#include "ColorCout.h"
#include <boost/multi_array.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <cmath>
#include "PolarVec.h"

//#define _USE_DEBUGINFO
//#define _CHECK_DEBUG

using namespace std;
using namespace boost::numeric;

int CJPDAFilter::_nTotalPFNum = 0;
bool CJPDAFilter::s_bUseRemovedHypoInWeighting = false;
double CJPDAFilter::_dFPFarProb = 0.05;
double CJPDAFilter::_dDetectLenMax = 15*1000;


CJPDAFilter::CJPDAFilter(size_t nParticleNum, size_t nStateDim) : P(nParticleNum, nStateDim) {
  _vParticleCenters.resize(nParticleNum);

  _nPFID = _nTotalPFNum;
  ++_nTotalPFNum;

  
  _nSeveralMatchedRangeThr = 5;

  _vStatusVector.resize(3);
  _vStatusVector[0] = 1;
  _vStatusVector[1] = 0;
  _vStatusVector[2] = 0;

  _bCrushWithLRF = false;
  SetInterval(1);
  _dExistanceRate = 1.0;
  _bSplittable = false;

  _dMatchDistThr=  180;
  
  _dOtherMOLenAve = 250;
  _dOtherMOLenStdDev = 50;
  _dSigma = 80;
  _dFPExistLen  = 200;
  _dFPNearProb  = 0.1;
  _dPassProbTotal = 0.001;
}

CJPDAFilter::~CJPDAFilter() {
}


void CJPDAFilter::SetLikelihoodFunctionParams(double  dSigma, double dOtherMOLenAve, double dOtherMOLenStdDev, double dFPExistLen, double dFPNearProb, double dPassProbTotal) {

  if (dSigma>0)            _dSigma            = dSigma;
  if (dOtherMOLenAve>0)    _dOtherMOLenAve    = dOtherMOLenAve;
  if (dOtherMOLenStdDev>0) _dOtherMOLenStdDev = dOtherMOLenStdDev;
  if (dFPExistLen>0)       _dFPExistLen       = dFPExistLen;
  if (dFPNearProb>0)       _dFPNearProb       = dFPNearProb;
  if (dPassProbTotal>0)    _dPassProbTotal    = dPassProbTotal;
}


void CJPDAFilter::Predict(const SJPDAFilterData &rData) {

  BoostVec v(3);
#ifdef _CHECK_DEBUG
  for (size_t n1 = 0; n1 < _avPredict.size(); ++n1) {
    const auto& rp = _avPredict[n1];
    if ( _isnan(rp(0)) || _isnan(rp(1))) {
      cout << __FUNCTION__ << "nan found at " << n1 <<  endl;
    }
  }
#endif

  const auto &pCo = rData._pLaserData->GetCo();
  const auto &rLRFPos = pCo->GetPos();
  double dDistTotal = 0;

  std::vector<BoostVec>::iterator itPredict = _avPredict.begin();
  std::vector<BoostVec>::const_iterator itFilter = _avFilter.begin();
  _vParticleCenters.resize(_nParticleNum);
  auto itPFCenter = _vParticleCenters.begin();
  _vPredictResult=boost::numeric::ublas::zero_vector<double>(_nStateDim);
  for(;itPredict!=_avPredict.end();++itPredict,++itFilter, ++itPFCenter) {
    Process(*itPredict, *itFilter);
    _vPredictResult+=(*itPredict);
    v(0) = (*itPredict)(0);
    v(1) = (*itPredict)(1);
    v(2) = 0;
    rData._pLaserData->GetCo()->VectorGlobalToLocal(v);
    (*itPFCenter) = v;
    dDistTotal += FastMath::fast_hypot(v(0), v(1));
  }
  _vPredictResult/=_nParticleNum;
  _dAverageDistFromLRF = dDistTotal/_nParticleNum;

#ifdef _CHECK_DEBUG
  for (size_t n1 = 0; n1 < _avFilter.size(); ++n1) {
    const auto& rp = _avFilter[n1];
    if ( _isnan(rp(0)) || _isnan(rp(1))) {
      ccout->SetColor(ColorCout::eRed);
      ccout << __FUNCTION__ << " nan found at: " << n1 << endl;
    }
  }
#endif

  UpdateRangeGroup(rData);
  _vpPBuffer.clear();
  _vHypos.clear();
  _vPValues.clear();
  SetInterval(1);

  //SetPointCorreBitsGroupで使う
  _CurrentData = rData;
  _vParticleDebugInfo.resize(_nParticleNum);
  _vParticleWeights.resize(_nParticleNum);

#ifdef _USE_DEBUGINFO
  for (auto it=_vParticleDebugInfo.begin(); it!=_vParticleDebugInfo.end(); ++it) {
    it->clear();
  }
#endif
  for (auto it=_vParticleWeights.begin(); it!=_vParticleWeights.end(); ++it) {
    it->clear();
  }
}


bool CJPDAFilter::IsPointInHypo(int nID, const RangeInterval &rRange) {

  if (in(nID, rRange)) return true;

  if (rRange.lower()<0) {
    if (in(nID-(int)_PointCorreBits.size(), rRange)) return true;
  }
  return false;
}

void CJPDAFilter::UpdateRangeGroup(const SJPDAFilterData &rData) {

  //out1: _vMatchedRangeSet      : Match判定になった点のリスト 今は使ってない
  //out2: _SeveralMatchedRangeID : 指定数以上のパーティクルがMatch判定になった点の範囲
  
  const boost::shared_ptr<const CLaserData> &pLaserData = rData._pLaserData;
  const std::vector<int> &rvPointStatus = rData._vPointStatus;
  const boost::shared_ptr<const CCascadedCoords> &pLRFCo = rData._pLaserData->GetCo();
  _LRFProperty = rData._pLaserData->GetProperty();
  BoostVec v(3);

  _PointCorreBits.resize(_LRFProperty._nElemNum);
  _PointCorreBits.reset();
  _vParticleLaserStatus.clear();
  _vParticleLaserStatus.resize(_nParticleNum);
  _vParticleCorrelations.clear();
  _vParticleCorrelations.resize(_nParticleNum);

  int nMatchedMin = INT_MAX;
  int nMatchedMax = INT_MIN;
  _SeveralMatchedRangeID.set_empty();
  _ParticlePointRange.set_empty();
  _vMatchedRangeSet.clear();

  vector<int> vnMatchedVote(pLaserData->GetProperty()._nElemNum, 0);
  bool bFirst = true;
  for (int i=0; i<(int)_nParticleNum; ++i) {
    auto &vnCorrelations = _vParticleCorrelations[i];
    if (CalcParticleCorrelation(rData, i, vnCorrelations)) {
      if (vnCorrelations.empty()) continue;
      RangeInterval PFRange = hull(vnCorrelations.front().first, vnCorrelations.back().first);
      _ParticlePointRange = hull(_ParticlePointRange, PFRange);
      for (size_t j=0; j<vnCorrelations.size(); ++j) {
        int n = vnCorrelations[j].first;
        int nStatus = rvPointStatus[n];
        double dDist = vnCorrelations[j].second-pLaserData->GetRawData()[n];
        if ((nStatus == 1) && (abs(dDist)<_dMatchDistThr)) {
          nMatchedMax = max(n, nMatchedMax);
          nMatchedMin = min(n, nMatchedMin);
          _vMatchedRangeSet.insert(n);
          ++vnMatchedVote[n];
        }
      }
    }
    else {
      _bCrushWithLRF = true;
    }
  }
  if (nMatchedMax != INT_MIN) {
    for (int n=nMatchedMin; n<=nMatchedMax; ++n) {
      if (vnMatchedVote[n] >= _nSeveralMatchedRangeThr) {
        _PointCorreBits[n] = 1;
      }
    }
  }

  //_SeveralMatchedRangeIDを更新．一周をまたいでいる時は負の値を返す
  if (_PointCorreBits[0] && _PointCorreBits[_PointCorreBits.size()-1]) {
    //cout << "isshuu detect!!" << endl;
    int nMin = -1;
    for (int i=(int)(_PointCorreBits.size()/2); i<(int)_PointCorreBits.size(); ++i) {
      if (_PointCorreBits[i]) {
        nMin = (int)i;
        break;
      }
    }
    int nMax = -1;
    for (int i=((int)(_PointCorreBits.size()/2))-1; i>=0; --i) {
      if (_PointCorreBits[i]) {
        nMax = (int)i;
        break;
      }
    }
    _SeveralMatchedRangeID = hull(nMin-(int)(_PointCorreBits.size()), nMax);
  }
  else {

    int nMin = -1;
    int nMax = -1;
    for (size_t i=0; i<_PointCorreBits.size(); ++i) {
      if (_PointCorreBits[i]) {
        nMin = (int)i;
        break;
      }
    }
    for (int i=((int)_PointCorreBits.size())-1; i>=0; --i) {
      if (_PointCorreBits[i]) {
        nMax = (int)i;
        break;
      }
    }
    if (nMin >= 0) {
      _SeveralMatchedRangeID = hull(nMin,nMax);
    }
    else _SeveralMatchedRangeID.set_empty();
  }
  _PointCorreBitsGroup = _PointCorreBits;
}

void CJPDAFilter::PrepareMemories(size_t nHypoTotal) {

#ifdef _USE_DEBUGINFO
  for (auto it=_vParticleDebugInfo.begin(); it!=_vParticleDebugInfo.end(); ++it) {
    it->reserve(nHypoTotal+1);
  }
#endif
  for (auto it=_vParticleWeights.begin(); it!=_vParticleWeights.end(); ++it) {
    it->reserve(nHypoTotal+1);
  }

}


bool IsHypoSame(const std::vector<CJPDAFilter::RangeInterval> &rHypo1, 
                const std::vector<CJPDAFilter::RangeInterval> &rHypo2)
{
  if (rHypo1.size() != rHypo2.size()) return false;
  for (size_t i=0; i<rHypo1.size(); ++i) {
    if ((rHypo1[i].lower() != rHypo2[i].lower())||(rHypo1[i].upper() != rHypo2[i].upper()))
      return false;
  }
  return true;

}

pair<double, int> CJPDAFilter::CalcP(const std::vector<RangeInterval> &rHypo) {

  //既に登録済みの場合はそれを利用
  for (size_t k=0; k<_vHypos.size(); ++k) {
    const auto &rvRegisterd = _vHypos[k];
    if (IsHypoSame(rHypo, rvRegisterd)){
      return make_pair(_vPValues[k], k);
    }
  }
  double dTotal = 0;
  boost::shared_ptr<std::vector<double>> pBuffer(new vector<double>(_nParticleNum));
  for (size_t n=0; n<_nParticleNum; ++n) {
#ifdef _USE_DEBUGINFO
    SParticleDebugInfo TempInfo;
    double dLikelihood = GetParticleLikelihood(n, rHypo, TempInfo);
    _vParticleDebugInfo[n].push_back(TempInfo);
#else
    double dLikelihood = GetParticleLikelihood(n, rHypo);
#endif
    _vParticleWeights[n].push_back(dLikelihood);
    dTotal += dLikelihood;
    (*pBuffer)[n] = dLikelihood;
  }
  dTotal /= _nParticleNum;
  if (dTotal <= DBL_MIN*10) dTotal = DBL_MIN*10; //0にしないようにする

  _vpPBuffer.push_back(pBuffer);
  _vHypos.push_back(rHypo);
  _vPValues.push_back(dTotal);
  return make_pair(dTotal, (int)_vpPBuffer.size()-1);
}


const BoostVec& CJPDAFilter::Update(const SJPDAFilterData &rData, const std::vector<std::pair<double, int>> &rvBeta) {
  _vBeta2 = rvBeta;
  
  //weighting
  vector<double> vWeightTotal(rvBeta.size(), 0.0);
  for (size_t i=0; i<_nParticleNum; ++i) {
    for (size_t j=0; j<rvBeta.size(); ++j) {
      const auto &rBeta = rvBeta[j];
      if (rBeta.second >= 0) {
        vWeightTotal[j]+=(*_vpPBuffer[rBeta.second])[i];;
      }
      else if (s_bUseRemovedHypoInWeighting) {
        vWeightTotal[j] = 1.0;
      }
    }
  }
  double dWeightSum = 0;
  for (size_t i=0; i<_nParticleNum; ++i) {
    _vWeight[i] = 0;
    for (size_t j=0; j<rvBeta.size(); ++j) {
      const auto &rBeta = rvBeta[j];
      if (rBeta.second >= 0) {
        if (vWeightTotal[j] > 0) { //ありえない仮説のweightは0になることがある
          double dWeight = (*_vpPBuffer[rBeta.second])[i];
          _vWeight[i] += dWeight/vWeightTotal[j]*rBeta.first;
        }
      }
      else if (s_bUseRemovedHypoInWeighting) {
        double dWeight = 1.0/_nParticleNum;
        _vWeight[i] += dWeight*rBeta.first;
      }
    }
    dWeightSum += _vWeight[i];
  }

  if (dWeightSum > 0) { //念のため
    for(auto itWeight=_vWeight.begin();itWeight!=_vWeight.end();++itWeight) {
      (*itWeight)/=dWeightSum;
    }
  }
  else { //どうしようもないので等分
    for(auto itWeight=_vWeight.begin();itWeight!=_vWeight.end();++itWeight) {
      (*itWeight) = 1.0/_vWeight.size();
    }
  }

  Resampling(rData);
  CalcResult();
//  UpdateStatusVector();
  return GetResult();
}

void CJPDAFilter::Resampling(const SJPDAFilterData &rvY) {

  P::Resampling(rvY);
}

void CJPDAFilter::CalcResult() {

  /*
  低速の時，逆方向の速度ベクトルを持つパーティクルのせいで平均がおかしくなるため，工夫が必要．
  速度ベクトルの向きががおかしいと楕円の方向も変わるので，正しい向きにしておく必要がある．

  1.最尤パーティクルを求める
  2.最尤パーティクルとの角度差の重み付け和を計算
  3.結果の速度ベクトルを最尤パーティクルの角度＋角度差の重み付け和を速度ベクトルの向きとして採用
  4.最尤パーティクルの速度ベクトルが180度逆の可能性もある．dGyakuに速度ベクトルの向きが90度以上違うパーティクルの尤度和を入れ，それが0.5を越えたら180度逆と判定．
  */

  double dMax = -1;
  size_t nMax = 0;
  for (size_t i=0; i<_vWeight.size(); ++i) {
    const double &rW = _vWeight[i];
    if (rW > dMax) {
      dMax = rW;
      nMax = i;
    }
  }
  CAngle aMax = FastMath::table_atan2(_avPredict[nMax](3),_avPredict[nMax](2));
  double dDiffTotal = 0;
  double dSin = 0;
  double dCos = 0;
  double dGyaku = 0;

  std::fill(_vResult.begin(), _vResult.end(), 0);
  std::vector<BoostVec>::const_iterator itPredict = _avPredict.begin();
  std::vector<double>::const_iterator itWeight = _vWeight.begin();  
  for(;itPredict!=_avPredict.end();++itPredict,++itWeight) {
    _vResult+=(*itWeight)*(*itPredict);
    CAngle dDiff = (aMax-FastMath::table_atan2((*itPredict)(3),(*itPredict)(2)));
    if (abs(dDiff.get()) > M_PI/2) {
      dGyaku += (*itWeight);
      dDiff += M_PI;
    }
    dDiffTotal += dDiff.get()*(*itWeight);
  }
  CAngle dAve1 = aMax-dDiffTotal;
  if (dGyaku > 0.5) {
    dAve1 += M_PI;
  }
  double dLen = FastMath::fast_hypot(_vResult(2), _vResult(3));
  _vResult(2) = dLen * FastMath::table_cos(dAve1.get());
  _vResult(3) = dLen * FastMath::table_sin(dAve1.get());
}

double CJPDAFilter::Likelihood(const BoostVec &rX, const SJPDAFilterData &rY) {
  return 0;
}

void CJPDAFilter::MakeCovarianceMatrix(BoostMat &rResult) const {

  rResult = boost::numeric::ublas::zero_matrix<double>(_nStateDim,_nStateDim);
  std::vector<BoostVec>::const_iterator itPredict = P::_avPredict.begin();
  std::vector<double>::const_iterator itWeight = P::_vWeight.begin();
  BoostVec vDiff(2);
  BoostVec vPF(2);
  BoostVec vCenter(2); vCenter(0) = _vResult(0); vCenter(1) = _vResult(1);
  for (; itPredict != P::_avPredict.end(); ++itPredict, ++itWeight) {
    vPF(0) = (*itPredict)(0);
    vPF(1) = (*itPredict)(1);
    vDiff = vPF-vCenter;
    for (size_t i=0; i<2; ++i) {
      rResult(i,i) += vDiff(i)*vDiff(i)*(*itWeight);
      for (size_t j=i+1; j<2; ++j) {
        double d = vDiff(i)*vDiff(j)*(*itWeight);
        rResult(i,j) += d;
        rResult(j,i) += d;
      }
    }
  }

}

boost::tuple<CAngle, double, double> CJPDAFilter::CalcErrorEllipse() const {
  //まず共分散行列を求める
  BoostMat mErrorMatrix;
  MakeCovarianceMatrix(mErrorMatrix);
  double b1 = -(mErrorMatrix(0,0)+mErrorMatrix(1,1));
  double c1 = mErrorMatrix(0,0)*mErrorMatrix(1,1)-(mErrorMatrix(1,0)*mErrorMatrix(0,1));
  double dXSigma = -b1 + sqrt(b1*b1-4*c1);
  double dYSigma = -b1 - sqrt(b1*b1-4*c1);

  CAngle dAngle = atan( (dXSigma-mErrorMatrix(0,0))/mErrorMatrix(1,0));

  return boost::make_tuple(dAngle, sqrt(dXSigma), sqrt(dYSigma));
}


double GetGaussA(double x, double dSigma) {
  return fmath::expd( (x*x)/(dSigma*dSigma)*-0.5);
}
const double dSq2 = sqrt(2.0);
double GetRuiseki(double x, double dAve, double dSD) {
  return 0.5*(1.0+boost::math::erf((x-dAve)/(dSq2*dSD)) );
}

#include <boost/math/special_functions/erf.hpp>
void CJPDAFilter::SetPointCorreBitsGroup(const boost::dynamic_bitset<> &rPointCorreBitsGroup) {

  int nFirst = 0;
  int nLast = 0;

  _PointCorreBitsGroup = rPointCorreBitsGroup;

  auto nFirstMatch = rPointCorreBitsGroup.find_first();
  auto nLastMatch = nFirstMatch;
  if (nFirstMatch != boost::dynamic_bitset<>::npos) {
    while(true) {
      int n = rPointCorreBitsGroup.find_next(nLastMatch);
      if (n == boost::dynamic_bitset<>::npos) break;
      nLastMatch = n;
    }
    nFirst = min( (int)nFirstMatch, _ParticlePointRange.lower());
    nLast = max( (int)nLastMatch, _ParticlePointRange.upper());
  }
  else {
    nFirst = _ParticlePointRange.lower();
    nLast = _ParticlePointRange.upper();
  }


  nFirst = (int)(ceil(nFirst/(double)(_nInterval))*_nInterval);
  double dS1 = sqrt(2*M_PI)*_dSigma;
  double dFPFarP = GetFalsePositiveProbDensity();

  try {

    for (size_t n=0; n<_nParticleNum; ++n) {
      auto &rvCurrentLaserStatus = _vParticleLaserStatus[n];
      rvCurrentLaserStatus.clear();
      const auto &rvCorrelation = _vParticleCorrelations[n];
      size_t nCorr = rvCorrelation.size()/_nInterval;
      if (rvCorrelation.empty()) nCorr = 1;
      double _dPassProb = pow(_dPassProbTotal, 1.0/nCorr); //TODO 範囲の端でサイズが小さくなる
      rvCurrentLaserStatus.resize( (nLast-nFirst)/_nInterval + 1);

      for (size_t n2=0; n2<rvCurrentLaserStatus.size(); ++n2) {
        int j=nFirst+_nInterval*n2;

        auto &rLaserStatus = rvCurrentLaserStatus[n2];
        rLaserStatus.nAngleRangeID = j;
        rLaserStatus.dLaserRange = _CurrentData._pLaserData->GetRawData()[j];
        if (rLaserStatus.dLaserRange == 0) 
          rLaserStatus.dLaserRange = _CurrentData._pLaserData->GetProperty()._dMaxRange;
        rLaserStatus.nPointStatus = _CurrentData._vPointStatus[j];

        rLaserStatus.dLikelihoodInHypo = 1;
        rLaserStatus.dLikelihoodOutofHypo = 1;
        rLaserStatus.dLikelihoodNotExist = 1;
        rLaserStatus.nInRangeStatus = SLaserStatus::eNOP;       
        rLaserStatus.nOutofRangeStatus = SLaserStatus::eNOP2;       
        //正：Laserが手前，Particle奥(OCC)  負：Laserが奥，Particle手前(PASS)

        if (!_PointCorreBitsGroup[j] && (rLaserStatus.nPointStatus==1)) { //グループ範囲外点は考える必要なし
          rLaserStatus.nPointStatus = 0;
        }
        if (!rvCorrelation.empty() && 
          (rvCorrelation.front().first <= j) && (j <= rvCorrelation.back().first)) {//パーティクル番号内
          rLaserStatus.bInParticleRange = true;
          //temp debug込み -> 実機で出る？
          int nCorrFirst = rvCorrelation.front().first;
          try {
            auto &rTemp = rvCorrelation.at(j-nCorrFirst);
            if (rTemp.first != j) {
              cout << __FUNCTION__ << " something wrong "  << rTemp.first << " " << j << endl;
            }
            rLaserStatus.dParticlePointRange = rTemp.second;
          }
          catch (std::exception &e) {
            cout << e.what() << endl;
          }
        }
        else {
          rLaserStatus.bInParticleRange = false;
          rLaserStatus.dParticlePointRange = -1;
        }

        double dDistDiff = rLaserStatus.dParticlePointRange - rLaserStatus.dLaserRange;

        if (rLaserStatus.nPointStatus == 1) { //移動
          const auto &pPoint = _CurrentData._pLaserData->GetPoints().at(j);
          double x = pPoint->GetX();
          double y = pPoint->GetY();
          rLaserStatus.dDistFromParticle = GetNearestDistFromParticle(n, x, y, _CurrentData._pLaserData->GetCo());

          //仮説範囲内
          if (rLaserStatus.bInParticleRange) { //Match
            rLaserStatus.dLikelihoodInHypo = GetGaussA(dDistDiff,_dSigma)/dS1;
            if (dDistDiff < _dFPExistLen) rLaserStatus.dLikelihoodInHypo += _dFPNearProb/_dFPExistLen;
            rLaserStatus.dLikelihoodInHypo += dFPFarP;
            rLaserStatus.nInRangeStatus = SLaserStatus::eMatch;;
            //すごく遠い時もある
            if (abs(dDistDiff) > 2.5*_dSigma)
              rLaserStatus.nInRangeStatus = SLaserStatus::eMatchFar;
          }
          else { //FP
            rLaserStatus.dLikelihoodInHypo = 0;
            if (rLaserStatus.dDistFromParticle < _dFPExistLen) {
              rLaserStatus.dLikelihoodInHypo += _dFPNearProb/_dFPExistLen;
            }
            rLaserStatus.dLikelihoodInHypo += dFPFarP;
            rLaserStatus.nInRangeStatus = SLaserStatus::eFP;
          }

          //仮説範囲外
          if (rLaserStatus.bInParticleRange) { //Pass or Occ
            if (dDistDiff <=0) { //PassThrough
              rLaserStatus.nOutofRangeStatus = SLaserStatus::eDynamicPT;
              rLaserStatus.dLikelihoodOutofHypo = _dPassProb;
            } 
            else { //occ
              rLaserStatus.nOutofRangeStatus = SLaserStatus::eDynamicOcc;
              rLaserStatus.dLikelihoodOutofHypo = GetRuiseki(dDistDiff, _dOtherMOLenAve, _dOtherMOLenStdDev);
            }
          }
          else { //NOP
            rLaserStatus.nOutofRangeStatus = SLaserStatus::eNOP2;       
          }
        }
        else if (rLaserStatus.nPointStatus == 0) { //静止
          if (rLaserStatus.bInParticleRange) {
            if (dDistDiff <=0)  { //PassThrough
              rLaserStatus.nInRangeStatus = SLaserStatus::eStaticPT;
              rLaserStatus.nOutofRangeStatus = SLaserStatus::eStaticPT2;
              rLaserStatus.dLikelihoodInHypo = _dPassProb;
              rLaserStatus.dLikelihoodOutofHypo = _dPassProb;
            }
            else { //Occlusion or パーティクル範囲外
              rLaserStatus.nInRangeStatus = SLaserStatus::eStaticOcc;
              rLaserStatus.nOutofRangeStatus = SLaserStatus::eStaticOcc2;
            }
          }
          else { 
            rLaserStatus.nInRangeStatus = SLaserStatus::eNOP;
            rLaserStatus.nOutofRangeStatus = SLaserStatus::eNOP2;
          }
        }
      }

    }
//    double dTime = mt.elapsed();
//    cout << "#" << GetPFID() << " time: " << dTime << endl;

    }
  catch (std::exception &e) {
    cout << "Error in " << __FUNCTION__ << ":" << e.what() << endl;
    throw;
  }
}

double CJPDAFilter::GetParticleLikelihood(size_t nPFNum, const std::vector<CJPDAFilter::RangeInterval> &rHypo, CJPDAFilter::SParticleDebugInfo &rDebugInfo) {

  rDebugInfo = SParticleDebugInfo();

  const BoostVec &rParticle = _avPredict[nPFNum];
  const vector<SLaserStatus> &rPFPoints = _vParticleLaserStatus.at(nPFNum);

  double dLikelihood = 1;

  for (auto itPoints = rPFPoints.begin(); itPoints != rPFPoints.end(); ++itPoints) {

    if (itPoints->nAngleRangeID < 0) { //間引き対象
      continue;
    }
    bool bInHypo = false;
    for (size_t n=0; n<rHypo.size(); ++n){
      if (IsPointInHypo(itPoints->nAngleRangeID, rHypo[n])) {
        bInHypo = true;
        break;
      }
    }
    if (bInHypo) {
      dLikelihood *= itPoints->dLikelihoodInHypo;
      ++rDebugInfo.vnPointStatus[(size_t)itPoints->nInRangeStatus];
    }
    else {
      dLikelihood *= itPoints->dLikelihoodOutofHypo;
      ++rDebugInfo.vnPointStatus[(size_t)itPoints->nOutofRangeStatus + (size_t)SLaserStatus::eInRangeTotal];
    }
  }

  if (!rPFPoints.empty()) {
    rDebugInfo.nRangeMax = rPFPoints.back().nAngleRangeID;
    rDebugInfo.nRangeMin = rPFPoints.front().nAngleRangeID;
  }

  return dLikelihood;
}

double CJPDAFilter::GetParticleLikelihood(size_t nPFNum, const std::vector<CJPDAFilter::RangeInterval> &rHypo) {

  const vector<SLaserStatus> &rPFPoints = _vParticleLaserStatus[nPFNum];
  double dLikelihood = 1;
  for (auto itPoints = rPFPoints.begin(); itPoints != rPFPoints.end(); ++itPoints) {
    if (itPoints->nAngleRangeID < 0) { //間引き対象
      continue;
    }
    bool bInHypo = false;
    for (size_t n=0; n<rHypo.size(); ++n){
      if (IsPointInHypo(itPoints->nAngleRangeID, rHypo[n])) {
        bInHypo = true;
        break;
      }
    }
    if (bInHypo) {
      dLikelihood *= itPoints->dLikelihoodInHypo;
    }
    else {
      dLikelihood *= itPoints->dLikelihoodOutofHypo;
    }
  }
  return dLikelihood;
}



std::vector<std::string> CJPDAFilter::SLaserStatus::s_vLaserString;
const std::string &CJPDAFilter::SLaserStatus::GetLaserStatusStr(size_t n) {

  if (s_vLaserString.empty()) {
    s_vLaserString.push_back("Match");
    s_vLaserString.push_back("FP");
    s_vLaserString.push_back("StaticOcc");
    s_vLaserString.push_back("StaticPT");
    s_vLaserString.push_back("MatchFar");
    s_vLaserString.push_back("NOP");

    s_vLaserString.push_back("DynamicOcc");
    s_vLaserString.push_back("DynamicPT");
    s_vLaserString.push_back("StaticOcc2");
    s_vLaserString.push_back("StaticPT2");
    s_vLaserString.push_back("NOP2");
  }

  if (n < s_vLaserString.size()) {
    return s_vLaserString[n];
  }
  throw std::logic_error("GetLaserStatusStr OutofRange");
}
