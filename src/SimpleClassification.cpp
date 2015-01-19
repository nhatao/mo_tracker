#include "StdAfx_MOTracking.h"
#include "SimpleClassification.h"

using namespace std;

static double GetGauss(double dDist1, double dSigma1) {
  return exp(dDist1*dDist1/(dSigma1*dSigma1)*-0.5)/sqrt(2*M_PI*dSigma1*dSigma1);
}


CSimpleClassification::CSimpleClassification(void)
{
  _dTotalLenMinThreshMultiplier = 2.0;
  _dTotalLenMaxThreshMultiplier = 2.0;
  _dDistBetweenEdgeMinThreshMultiplier = 2.0;
  _dDistBetweenEdgeMaxThreshMultiplier = 2.0;

  SParams OnePerson = {753.824, 201.727, 475.951, 81.4371};
  SParams TwoPerson = {1524.10, 250.039, 923.895, 159.391};
  _vParams.push_back(OnePerson);
  _vParams.push_back(TwoPerson);
}


CSimpleClassification::~CSimpleClassification(void)
{
}

boost::shared_ptr<SClassificationResult> CSimpleClassification::Proc(const std::vector<boost::shared_ptr<CJPDATracker::SPointCluster>> &rvpCluster) {

  double dTotalLenMin = _vParams.front().dTotalLenAverage - _dTotalLenMinThreshMultiplier * _vParams.front().dTotalLenSigma;
  double dTotalLenMax = _vParams.back().dTotalLenAverage + _dTotalLenMinThreshMultiplier * _vParams.back().dTotalLenSigma;

  double dEdgeLenMin = _vParams.front().dDistBetweenEdgeAverage - _dDistBetweenEdgeMinThreshMultiplier * _vParams.front().dDistBetweenEdgeSigma;
  double dEdgeLenMax = _vParams.back().dDistBetweenEdgeAverage + _dDistBetweenEdgeMaxThreshMultiplier * _vParams.back().dDistBetweenEdgeSigma;

  boost::shared_ptr<SClassificationResult> pResult(new SClassificationResult);
  if (rvpCluster.size() != 1) { //currently not implemented
    return pResult;
  }
  if (rvpCluster[0]->_vPoints.size() < 3) {
    return pResult;
  }

  double dTotalLen = rvpCluster[0]->_dTotalLen;
  double dDistBetweenEdge = rvpCluster[0]->_dDistBetweenEdge;
  double dWidth = rvpCluster[0]->_dWidthFromLRF;

  if ((dTotalLen < dTotalLenMin) || (dTotalLenMax < dTotalLen)) {
    return pResult;
  }
  if ((dDistBetweenEdge < dEdgeLenMin) || (dEdgeLenMax < dDistBetweenEdge)) {
    return pResult;
  }

  //直線型のFalse Positiveを見分ける
  if (dDistBetweenEdge/dWidth > 1.95) {
    return pResult;
  }

  vector<double> vScore;
  for (size_t i=0; i<_vParams.size(); ++i) {
    double dLenScore = GetGauss(_vParams[i].dTotalLenAverage-dTotalLen, _vParams[i].dTotalLenSigma);
    double dEdgeScore = GetGauss(_vParams[i].dDistBetweenEdgeAverage-dDistBetweenEdge, _vParams[i].dDistBetweenEdgeSigma);
    vScore.push_back(dLenScore*dEdgeScore);
  }

  //nMaxPos+1 = 人数
  size_t nMaxPos = std::max_element(vScore.begin(), vScore.end()) - vScore.begin();

  /*
  cout << "OK: " << dDistBetweenEdge/dWidth << endl;
  cout << "TotalLen: " << dTotalLen << endl;
  cout << "dDistBetweenEdge: " << dDistBetweenEdge << endl;
  cout << "dWidth: " << dWidth << endl;
  cout << "MaxPos: " << nMaxPos << endl;
  */
  int nIDMin = rvpCluster[0]->_PointIDRange.lower();
  int nIDMax = rvpCluster[0]->_PointIDRange.upper();
  int nFirstPos = 0;
  double dCurLen = 0;
  double dDistThr = dTotalLen/(nMaxPos+1);

  //クラスタを等分に分割
  for (int i=0; i<(int)rvpCluster[0]->_vPoints.size()-1; ++i) {
    const auto &p1 = rvpCluster[0]->_vPoints[i];
    const auto &p2 = rvpCluster[0]->_vPoints[i+1];
    double dDist = _hypot(p1->GetLocalX()-p2->GetLocalX(),p1->GetLocalY()-p2->GetLocalY()); 
    dCurLen += dDist;
    if ((dCurLen > dDistThr) || (i == rvpCluster[0]->_vPoints.size()-2)) {   
      SClassificationResult::SMOStatus s;
      s.dProb = 0.8;
      int nMax = i;
      if (i == rvpCluster[0]->_vPoints.size()-2) {
        nMax = rvpCluster[0]->_vPoints.size();
      }
      for (int n=nFirstPos; n<nMax; ++n) {
        s.vIDs.push_back(n+nIDMin);
      }
      pResult->vPersons.push_back(s);
      nFirstPos = i;
      dCurLen = 0;
    }
  }

  /*
  cout << nIDMin << " " << nIDMax << endl;
  for (size_t i=0; i<pResult->vPersons.size(); ++i) {
    cout << "#" <<i << " " << pResult->vPersons[i].vIDs.front() << ":" << pResult->vPersons[i].vIDs.back() << endl;
  }
  */
  return pResult;
}
