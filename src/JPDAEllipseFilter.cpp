#include "StdAfx_MOTracking.h"
#include "JPDAEllipseFilter.h"
#include "FastMath.h"
#include "ColorCout.h"
#include <boost/tuple/tuple.hpp>
#include "PolarVec.h"

using namespace std;

CJPDAEllipseFilter::CJPDAEllipseFilter(size_t nParticleNum)  : CJPDAFilter(nParticleNum, 6) {

  //DHRCの人体寸法データベースより作成
  _dR1Ave = 215.2/2+25.0;
  _dR2Ave = 428.9/2+25.0;
  _dR1StdDev = 19.6/2;
  _dR2StdDev = 32.0/2;

  _dR1Min = _dR1Ave-_dR1StdDev*2.5;
  _dR1Max = _dR1Ave+_dR1StdDev*2.5;
  _dR2Min = _dR2Ave-_dR2StdDev*2.5;
  _dR2Max = _dR2Ave+_dR2StdDev*2.5;

  double dX1 = sqrt(0.1)/0.1;
  _dXN = 250 /dX1;
  _dRadN = 2 /dX1;
  _dVN1 = 3000/dX1;
  _dRVN1 = Deg2Rad(60/dX1);
  _dVN2 = 1000/dX1;
  _dRVN2 = Deg2Rad(120/dX1);
  _dLowVelThr = 500;
  _dRVNLowVel = Deg2Rad(60);

  SetTimeStep(0.1);
  _nCnt = 0;
}

void CJPDAEllipseFilter::SetTimeStep(double dTimeStep) {
  _dTimeStep = dTimeStep;
}

void CJPDAEllipseFilter::ResetParticles(const SJPDAFilterData& rData, const std::vector<int> &rvIDs) {

  if (rvIDs.empty()) {
    ostringstream oss; oss << " rvIDs Emtpy in " << __FUNCTION__;
    throw std::logic_error(oss.str().c_str());
  }

  int nIDLower = rvIDs.front();
  int nIDUpper = rvIDs.back();
  int nMiddle = (nIDLower+nIDUpper)/2;
  CAngle dAveAngle = rData._pLaserData->GetProperty().IndexToAngle(nMiddle);

  double dDistAve = 0;
  const auto &rvDists = rData._pLaserData->GetRawData();
  const auto &rvPointStatus = rData._vPointStatus;
  int nPointTotal = 0;

  for (auto it=rvIDs.begin(); it!=rvIDs.end(); ++it) {
    if (rvPointStatus[*it] == 1) { //前景
      ++nPointTotal;
      dDistAve += rvDists[*it];
    }
  }

  if (nPointTotal == 0) {
    ostringstream oss; oss << __FUNCTION__ << " points all invalid";
    throw std::logic_error(oss.str().c_str());
  }

  dDistAve /= nPointTotal;

  _vResult = boost::numeric::ublas::zero_vector<double>(_nStateDim);
  BoostVec v(3);
  
  for (int n=0; n<(int)_nParticleNum; ++n) {

    double dR1n = _dR1Ave + _dR1StdDev*Random();
    double dR2n = _dR2Ave + _dR2StdDev*Random();

    dR1n = max(_dR1Min, min(dR1n, _dR1Max));
    dR2n = max(_dR2Min, min(dR2n, _dR2Max));

    double dDist = dDistAve + (dR1n+dR2n)/2;
//    double dDist = dDistAve;
    CAngle dAngle = dAveAngle + Deg2Rad(0.5)*Random();

    v(0) = dDist*cos(dAngle);
    v(1) = dDist*sin(dAngle);
    v(2) = 0;
    rData._pLaserData->GetCo()->VectorLocalToGlobal(v);

    _vParticleCenters[n].SetDist(dDist);
    _vParticleCenters[n].SetAngle(dAngle);
    auto &v1 = _avFilter[n];
    v1.resize(6);
    v1(0) = v(0);
    v1(1) = v(1);
    v1(2) = 1000*Random();
    v1(3) = 1000*Random();
    v1(4) = dR1n;
    v1(5) = dR2n;
    _vResult += v1;
  }

  _avPredict = _avFilter;
  _vResult/=_nParticleNum;

  std::fill(_vWeight.begin(), _vWeight.end(), 1.0/_vWeight.size());
}

void CJPDAEllipseFilter::Process(BoostVec& rvX, const BoostVec& rvXBefore) {

  double dTimeSqrt = sqrt(_dTimeStep);

  rvX = rvXBefore;

  double dDV = 0;
  double dDR = 0;

  //現在の速度
  double dV = FastMath::fast_hypot(rvX(2), rvX(3));
  CAngle dR(FastMath::table_atan2(rvX(3), rvX(2)));

  if (_nCnt%2 == 0) {
    //加減速するモデル
    dDV = _dVN1*dTimeSqrt*Random();
    dDR = _dRVN1*dTimeSqrt*Random()/2;
  }
  else {
    //旋回するモデル
    dDV = _dVN2*dTimeSqrt*Random();
    dDR = _dRVN2*dTimeSqrt*Random()/2;
  }
  dV += dDV;
  if (dV<0) {
    dV=-dV;
    dR+=M_PI;
  }
  if (dV<_dLowVelThr) {//特に低速域における速度変化への対応
    dDR += _dRVNLowVel*dTimeSqrt*Random();
  }

  dR += dDR;

  rvX(0) += dV*FastMath::table_cos(dR.get())*_dTimeStep;
  rvX(1) += dV*FastMath::table_sin(dR.get())*_dTimeStep;
  rvX(0) += Random()*_dXN*dTimeSqrt;
  rvX(1) += Random()*_dXN*dTimeSqrt;

  rvX(2) = dV*FastMath::table_cos(dR.get());
  rvX(3) = dV*FastMath::table_sin(dR.get());


  rvX(4) += Random()*_dRadN;
  rvX(5) += Random()*_dRadN;
  rvX(4) = max(_dR1Min, min(_dR1Max, rvX(4)));
  rvX(5) = max(_dR2Min, min(_dR2Max, rvX(5)));

  ++_nCnt;

  _vParticleEdgeInfo.resize(_nParticleNum);
}


std::pair<bool, double> GetDist(const CAngle& d, double r1, double r2, double a, double b) {

  double s1 = FastMath::table_sin(d.get());
  double c1 = FastMath::table_cos(d.get());

//  cout << d.get_deg() << "/" << c1 << endl;
  if (abs(c1) < 1.0e-24) {
    //x=0のとき
    double D=(1-a*a/r1/r1)*r2*r2;
    if (D>=0) {
      return make_pair(true, min(abs(b-sqrt(D)),abs(b+sqrt(D))));
    }
    else {
      return std::make_pair(false, 0);
    }
  }
  else {
    double m = s1 / c1;
    double w = r2/r1;
    double A = m*m+w*w;
    double B = -2*m*b-2*a*w*w;
    double C = b*b-r1*w*r1*w+a*w*a*w;
    double D = B*B-4*A*C;
    if (D>=0) {

      double d2 = sqrt((B*B-2*abs(B)*sqrt(D)+D)*(1+m*m)/(4*A*A));
      return std::make_pair(true, d2);
    }
    else {
      return std::make_pair(false, 0);
    }
  }
}

//http://www17.ocn.ne.jp/~lite/pro97doc.html
bool CJPDAEllipseFilter::CalcParticleCorrelation(const SJPDAFilterData &rData, int nPFNum, std::vector<std::pair<int, double>> &rvCorrelations) {

#ifdef MEASURETIME
  static int nCnt = 0;
  static double dTimeTotal1 = 0;
  static double dTimeTotal2 = 0;
  static double dTimeTotal3 = 0;
  mmtimer mt1;
#endif

  const auto &pLaserCo = rData._pLaserData->GetCo();
  const auto &vPosGlobal = _avPredict.at(nPFNum);
  const auto &rProp = rData._pLaserData->GetProperty();
  double dThetaGlobal = FastMath::table_atan2(vPosGlobal(3), vPosGlobal(2));
  CAngle dTheta =  dThetaGlobal - pLaserCo->GetYaw(); //dTheta:楕円回転角

  SParticleEdgeInfo &rEdge = _vParticleEdgeInfo.at(nPFNum);
  rEdge.Init();
  double dCenterFromLRF(FastMath::table_atan2(vPosGlobal(1)-pLaserCo->GetPos()(1),
                                              vPosGlobal(0)-pLaserCo->GetPos()(0)));
  rEdge._dMatchRangeMin = dCenterFromLRF+M_PI/2;
  rEdge._dMatchRangeMax = dCenterFromLRF-M_PI/2;

  double x1 = vPosGlobal(4)*FastMath::table_cos(dCenterFromLRF-M_PI/2-dThetaGlobal);
  double y1 = vPosGlobal(5)*FastMath::table_sin(dCenterFromLRF-M_PI/2-dThetaGlobal);
  double x2 = vPosGlobal(4)*FastMath::table_cos(dCenterFromLRF+M_PI/2-dThetaGlobal);
  double y2 = vPosGlobal(5)*FastMath::table_sin(dCenterFromLRF+M_PI/2-dThetaGlobal);
  double c1 = FastMath::table_cos(dThetaGlobal);
  double s1 = FastMath::table_sin(dThetaGlobal);

  rEdge._dLeftEdgeX  = vPosGlobal(0)+x1*c1-y1*s1;
  rEdge._dLeftEdgeY  = vPosGlobal(1)+x1*s1+y1*c1;
  rEdge._dRightEdgeX = vPosGlobal(0)+x2*c1-y2*s1;
  rEdge._dRightEdgeY = vPosGlobal(1)+x2*s1+y2*c1;

  const auto &R2 = pLaserCo->GetRot();
  const auto &P2 = pLaserCo->GetPos();
  double dX = R2(0,0)*vPosGlobal(0) + R2(1,0)*vPosGlobal(1) - R2(0,0)*P2(0) - R2(1,0)*P2(1);
  double dY = R2(1,1)*vPosGlobal(1) + R2(0,1)*vPosGlobal(0) - R2(1,1)*P2(1) - R2(0,1)*P2(0);

  double R00 = FastMath::table_cos(dTheta.get());
  double R01 = -FastMath::table_sin(dTheta.get());
  double R10 = FastMath::table_sin(dTheta.get()); 
  double R11 = FastMath::table_cos(dTheta.get());
  double a = R00*dX + R10*dY;
  double b = R11*dY + R01*dX;

#ifdef MEASURETIME
  dTimeTotal1 += mt1.elapsed(); mt1.restart();
#endif

  double r1 = vPosGlobal(4); //軸1
  double r2 = vPosGlobal(5); //軸2

  //LRFが円内にあったらNG めんどいので長軸だけ
  if ( (a*a+b*b) < max(r1*r1, r2*r2)) {
    return false;
  }
  double A1 = a*a-r1*r1;
  double B1 = -a*b;
  double C1 = b*b-r2*r2;

  double D1 = B1*B1-A1*C1;
  if (D1 < 0) {
    std::logic_error("something wrong: no sessen!");
  }
  double DD1 = sqrt(D1);
  double m1 = (-B1+DD1)/A1;
  double m2 = (-B1-DD1)/A1;

  CAngle dLargeAngle = dTheta+(FastMath::table_atan2(m1,1)); //large
  CAngle dSmallAngle = dTheta+(FastMath::table_atan2(m2,1)); //small

  //内積が負だったら180Deg足す
  double dN1 = 1*a + m1*b;
  double dN2 = 1*a + m2*b;
  if (dN1 < 0) dLargeAngle+=M_PI;
  if (dN2 < 0) dSmallAngle+=M_PI;

  int nMinAngle = (int)ceil(rProp.AngleToContinuousIndex(dSmallAngle));
  int nMaxAngle = (int)floor(rProp.AngleToContinuousIndex(dLargeAngle));

  bool bMinInRange = dSmallAngle.is_sandwiched(rProp._dFirstAngle, rProp.GetLastAngle());
  bool bMaxInRange = dLargeAngle.is_sandwiched(rProp._dFirstAngle, rProp.GetLastAngle());

  if (bMinInRange && bMaxInRange) {
    if (nMaxAngle < nMinAngle) {
      //一周した場合(これ以外は通常)
//      cout << "isshuu!" << endl;
      nMinAngle -= rProp._nElemNum;
    }
  }
  else if (bMinInRange) { //最後の方が範囲外 
    nMaxAngle = rProp._nElemNum-1;
  }
  else if (bMaxInRange) { //最初の方が範囲外
    nMinAngle = 0;
  }
  else { //全部範囲外, 死角にいる
    return true;
  }


  if (nMinAngle > nMaxAngle) {
    cout << __FUNCTION__ << " nMin/nMax wrong " << nMinAngle << " " << nMaxAngle << endl;
    cout << " stride: " << bMinInRange << "/" << bMaxInRange << endl;
    cout << " raw range:" << rProp.AngleToContinuousIndex(dSmallAngle) << " " << rProp.AngleToContinuousIndex(dLargeAngle) << endl;
    return false;
  }

#ifdef MEASURETIME
  dTimeTotal2 += mt1.elapsed(); mt1.restart();
#endif

  for (int n2=nMinAngle; n2<=nMaxAngle; ++n2) {
    int n = n2;
    if (n<0) n+=rProp._nElemNum;
    
    CAngle d = rProp.IndexToAngle(n) - dTheta.get();
    bool bOK;
    double dDist;
    boost::tie(bOK, dDist) = GetDist(d, r1, r2, a, b);
    if (bOK) rvCorrelations.push_back(make_pair(n, dDist)); //計算誤差のため範囲の端でbOKがfalseになることもある
  }

#ifdef MEASURETIME
  dTimeTotal3 += mt1.elapsed(); mt1.restart();
  //処理速度：dTimeTotal3が最も遅い
  ++nCnt;
  if (nCnt == 300) {
    cout << "Times: ";
    cout << dTimeTotal1 << " " << dTimeTotal2 << " " << dTimeTotal3 << endl;
    dTimeTotal1 = 0;
    dTimeTotal2 = 0;
    dTimeTotal3 = 0;
    nCnt = 0;
  }
#endif

  return true;

}


//double CJPDAEllipseFilter::GetNearestDistFromParticle(const SJPDAFilterData &rData, int nPFNo, double x, double y) {
double CJPDAEllipseFilter::GetNearestDistFromParticle(int nPFNo, double x, double y, const boost::shared_ptr<const CCascadedCoords> &pLRFCo) {

  const auto &vPosGlobal = _avPredict.at(nPFNo);
  const auto &rEdge = _vParticleEdgeInfo[nPFNo];

  double a = vPosGlobal(0);
  double b = vPosGlobal(1);
  double r1 = vPosGlobal(4); //軸1
  double r2 = vPosGlobal(5); //軸2

  CAngle dAngle = FastMath::table_atan2(y-b, x-a);

  //_vParticleEdgeInfoは既にLRFの位置を考慮しているため，pLRFCoを使う必要はない
  if (dAngle.is_sandwiched(rEdge._dMatchRangeMin, rEdge._dMatchRangeMax)) {
    double dXNearest = r1*FastMath::table_cos(dAngle.get());
    double dYNearest = r2*FastMath::table_sin(dAngle.get());
    double dNearestDist = FastMath::fast_hypot(dXNearest, dYNearest);
    double dDistFromCenter2 = FastMath::fast_hypot(a-x, b-y);
    return dDistFromCenter2 - dNearestDist;
  }
  else {

    double d1 = FastMath::fast_hypot(rEdge._dLeftEdgeX-x, rEdge._dLeftEdgeY-y);
    double d2 = FastMath::fast_hypot(rEdge._dRightEdgeX-x, rEdge._dRightEdgeY-y);

    return min(d1, d2);
  }
}

static void SET_VAL(const double &x, double &_x) {if(x>0) _x = x;}

void CJPDAEllipseFilter::SetSizeValues(double dR1Ave, double dR1StdDev, double dR1Max, double dR1Min,
  double dR2Ave, double dR2StdDev, double dR2Max, double dR2Min) {

  SET_VAL(dR1Ave, _dR1Ave);
  SET_VAL(dR1StdDev, _dR1StdDev);
  SET_VAL(dR1Max, _dR1Max); 
  SET_VAL(dR1Min, _dR1Min);
  SET_VAL(dR2Ave, _dR2Ave);
  SET_VAL(dR2StdDev, _dR2StdDev);
  SET_VAL(dR2Max, _dR2Max);
  SET_VAL(dR2Min, _dR2Min);
}

void CJPDAEllipseFilter::SetNoiseValues(double dXN, double dRadN, double dVN1, double dRVN1, double dVN2, double dRVN2, double dLowVelThr, double dRVNLowVel) {

  SET_VAL(dXN, _dXN);
  SET_VAL(dRadN, _dRadN);
  SET_VAL(dVN1, _dVN1);
  SET_VAL(dRVN1, _dRVN1);
  SET_VAL(dVN2, _dVN2);
  SET_VAL(dRVN2, _dRVN2);
  SET_VAL(dLowVelThr, _dLowVelThr);
  SET_VAL(dRVNLowVel, _dRVNLowVel);

}
