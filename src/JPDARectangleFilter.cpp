#include "StdAfx_MOTracking.h"
#include "JPDARectangleFilter.h"
#include "FastMath.h"
#include "ColorCout.h"
#include "CLine.h"
#include "PolarVec.h"

using namespace std;


CJPDARectangleFilter::CJPDARectangleFilter(size_t nParticleNum)  : CJPDAFilter(nParticleNum, 6) {

  _dMinWidth = 1000;
  _dMaxWidth = 2000;
  _dMinLength = 1500;
  _dMaxLength = 5000;

  _dXN = 200;
  _dVN = 1000;
  _dRVN = Deg2Rad(10);
  _dL1N = 50;
  _dL2N = 50;

  _dMatchDistThr = 300;
  _dSigma = 100;

  SetTimeStep(0.1);
  SetSplittable(true);
}


void CJPDARectangleFilter::SetTimeStep(double dTimeStep) {

  double dTime2 = min(0.3, dTimeStep);

  _dTimeStep = dTimeStep;
  BoostVec vNoise(_nStateDim);
  BoostVec vMNoise = boost::numeric::ublas::zero_vector<double>(_nStateDim);

  vNoise.resize(6);
  vNoise(0) = _dXN * dTime2;
  vNoise(1) = _dXN * dTime2;
  vNoise(2) = 0;
  vNoise(3) = 0;
  vNoise(4) = _dL1N * dTime2;
  vNoise(5) = _dL2N * dTime2;
  vMNoise(0) = _dSigma; 
  vMNoise(1) = _dSigma;
  _dVNoise1 = _dVN*dTime2;
  _dWNoise1 = _dRVN*dTime2;
  SetSystemNoise(vNoise);
  SetMeasurementNoise(vMNoise);
}


void CJPDARectangleFilter::GetResultPolygonal(CPolygonalRegion &r, double dEnlargeLen) {

  std::vector<BoostVec> vPoints;
  GetResultPoints(vPoints, dEnlargeLen);
  for (auto it=vPoints.begin(); it != vPoints.end(); ++it) {
    r.AddPoint((*it)(0), (*it)(1));
  }
}

void CJPDARectangleFilter::GetResultPoints(std::vector<BoostVec> &rvPoints, double dEnlargeLen) {

  CAngle a1 = atan2(_vResult(3), _vResult(2));
  CCoordinates2D CoCenter(_vResult(0), _vResult(1), a1.get());

  double dL1 = _vResult(4)/2 + dEnlargeLen;
  double dL2 = _vResult(5)/2 + dEnlargeLen;

  BoostVec v1(2);
  v1(0) =  dL1;
  v1(1) =  dL2;
  CoCenter.VectorLocalToGlobal(v1);
  rvPoints.push_back(v1);
  v1(0) =  dL1;
  v1(1) = -dL2;
  CoCenter.VectorLocalToGlobal(v1);
  rvPoints.push_back(v1);
  v1(0) = -dL1;
  v1(1) = -dL2;
  CoCenter.VectorLocalToGlobal(v1);
  rvPoints.push_back(v1);
  v1(0) = -dL1;
  v1(1) =  dL2;
  CoCenter.VectorLocalToGlobal(v1);
  rvPoints.push_back(v1);
}

void CJPDARectangleFilter::ResetParticles(const CCoordinates3D &rLRFCo, const BoostVec &rInitPos) {

  double _dAngleNoise = Deg2Rad(1.0);
  double _dVelNoise = 200;
  double _dLengthNoise = 20.0;
  double _dPosNoise = 100.0;

  double dInitVel = FastMath::fast_hypot(rInitPos(3), rInitPos(2));
  double dInitAngle = FastMath::table_atan2(rInitPos(3), rInitPos(2)); 

  _vResult = boost::numeric::ublas::zero_vector<double>(_nStateDim);
  for (int n=0; n<(int)_nParticleNum; ++n) {
    auto &v1 = _avFilter[n];
    v1(0) = rInitPos(0)+_dPosNoise*Random();
    v1(1) = rInitPos(1)+_dPosNoise*Random();
    v1(4) = rInitPos(4)+_dLengthNoise*Random();
    v1(5) = rInitPos(5)+_dLengthNoise*Random();
    double dVel = dInitVel + _dVelNoise*Random();
    double dAngle = dInitAngle + _dAngleNoise*Random();
    if (dVel < 0) {
      dVel = -dVel;
      dAngle += M_PI;
    }
    v1(2) = dVel * cos(dAngle);
    v1(3) = dVel * sin(dAngle);
    _vResult += v1;

    BoostVec vLocalCenter;
    BoostVec vCenter2(3);
    vCenter2(0) = v1(0);
    vCenter2(1) = v1(1);
    vCenter2(2) = 0;
    rLRFCo.VectorGlobalToLocal(vCenter2, vLocalCenter);
    _vParticleCenters[n].SetDist(FastMath::fast_hypot(vLocalCenter(0), vLocalCenter(1)));
    _vParticleCenters[n].SetAngle(FastMath::table_atan2(vLocalCenter(1), vLocalCenter(0)));
  }

  _avPredict = _avFilter;
  _vResult/=_nParticleNum;

  std::fill(_vWeight.begin(), _vWeight.end(), 1.0/_vWeight.size());

}

void CJPDARectangleFilter::ResetParticles(const SJPDAFilterData& rData, const std::vector<int> &rvIDs, const SBBInfo &rBBInfo) {

  if (rvIDs.empty()) {
    ostringstream oss; oss << " rvIDs Emtpy in " << __FUNCTION__;
    throw std::logic_error(oss.str().c_str());
  }

  BoostVec v1(2);
  vector<boost::shared_ptr<CCascadedVec> > vPoints;
  for (auto it=rvIDs.begin(); it!=rvIDs.end(); ++it) {
    const auto &vPos = rData._pLaserData->GetPoints().at(*it);
    const auto &rStatus = rData._vPointStatus.at(*it);
    if (rStatus == 1) {
      vPoints.push_back(vPos);
    }
  }

  //長い方を求める
  double dX1 = rBBInfo.vBBPos[0](0) - rBBInfo.vBBPos[1](0);
  double dY1 = rBBInfo.vBBPos[0](1) - rBBInfo.vBBPos[1](1);
  double dLen1 = FastMath::fast_hypot(dX1, dY1);
  double dX2 = rBBInfo.vBBPos[2](0) - rBBInfo.vBBPos[1](0);
  double dY2 = rBBInfo.vBBPos[2](1) - rBBInfo.vBBPos[1](1);
  double dLen2 = FastMath::fast_hypot(dX2, dY2);

  CAngle dLongAngle;
  double dLongLen;
  double dShortLen;
  if (dLen1 > dLen2) {
    dLongLen = dLen1;
    dShortLen = dLen2;
    dLongAngle = atan2(dY1, dX1);
  }
  else {
    dLongLen = dLen2;
    dShortLen = dLen1;
    dLongAngle = atan2(dY2, dX2);
  }

  BoostVec vCenter = (rBBInfo.vBBPos[0] + rBBInfo.vBBPos[2])/2;
  const auto &pLRFCo3D = rData._pLaserData->GetCo();
  boost::shared_ptr<CCoordinates2D> pLRFCo(new CCoordinates2D());

  pLRFCo->SetPos(pLRFCo3D->GetPos()(0), pLRFCo3D->GetPos()(1));
  pLRFCo->SetRotation(pLRFCo3D->GetYaw());

  double dRealLongLen = 4500;
  double dRealShortLen = 1800;

  double dXC = vCenter(0);
  double dYC = vCenter(1);

  int nType=0;

  //場合分け
  if (dLongLen > 2000) {
    //長い方が見えている
    if (dShortLen > 1200) {
      //短いほうが見えている
//      cout << "type1" << endl;
      //そのまま
      nType = 1;
    }
    else {
      //短いほうが見えてない
      nType = 2;
//      cout << "type2 "<< endl;
      CCoordinates2D CoCenter(dXC, dYC, dLongAngle.get());
      double d2 = dRealShortLen/2 - dShortLen/2;
      BoostVec v1(2); v1(0)=0; v1(1)=d2;
      BoostVec v2(2); v2(0)=0; v2(1)=-d2;
      CoCenter.TransformVector(v1);
      CoCenter.TransformVector(v2);

      BoostVec v11(2);
      BoostVec v21(2);
      pLRFCo->InverseTransformVector(v1, v11);
      pLRFCo->InverseTransformVector(v2, v21);
      double dDist1 = v11(0)*v11(0)+v11(1)*v11(1);
      double dDist2 = v21(0)*v21(0)+v21(1)*v21(1);
  //    cout << "(" << dXC << "," << dYC << ") -> ";
      if (dDist1 > dDist2) {
        dXC = v1(0);
        dYC = v1(1);
      }
      else {
        dXC = v2(0);
        dYC = v2(1);
      }
    }
  }
  else {
    //長い方が見えてない
    nType = 3;

    CCoordinates2D CoCenter(dXC, dYC, dLongAngle.get());
    double d2 = dRealLongLen/2 - dShortLen/2;
    BoostVec v1(2); v1(0)=0; v1(1)=d2;
    BoostVec v2(2); v2(0)=0; v2(1)=-d2;
    CoCenter.TransformVector(v1);
    CoCenter.TransformVector(v2);

    BoostVec v11(2);
    BoostVec v21(2);
    pLRFCo->InverseTransformVector(v1, v11);
    pLRFCo->InverseTransformVector(v2, v21);
    double dDist1 = v11(0)*v11(0)+v11(1)*v11(1);
    double dDist2 = v21(0)*v21(0)+v21(1)*v21(1);
    if (dDist1 > dDist2) {
      dXC = v1(0);
      dYC = v1(1);
    }
    else {
      dXC = v2(0);
      dYC = v2(1);
    }
    dLongAngle += M_PI/2;
  }

  _vResult = boost::numeric::ublas::zero_vector<double>(_nStateDim);

  for (int n=0; n<(int)_nParticleNum; ++n) {
    BoostVec vCenter2(2);
    vCenter2(0) = dXC+100*Random();
    vCenter2(1) = dYC+100*Random();

    BoostVec vLocalCenter;
    pLRFCo->VectorGlobalToLocal(vCenter2, vLocalCenter);
    double dDist = FastMath::fast_hypot(vLocalCenter(0), vLocalCenter(1));
    double dAngle = FastMath::table_atan2(vLocalCenter(1), vLocalCenter(0));

    _vParticleCenters[n].SetDist(dDist);
    _vParticleCenters[n].SetAngle(dAngle);

    auto &v1 = _avFilter[n];
    v1.resize(6);
    v1(0) = vCenter2(0);
    v1(1) = vCenter2(1);
    double dPAngle = dLongAngle.get() + Deg2Rad(2)*Random();
    v1(2) = cos(dPAngle);
    v1(3) = sin(dPAngle);
    v1(4) = dRealLongLen;
    v1(5) = dRealShortLen;
    _vResult += v1;
  }

  _avPredict = _avFilter;
  _vResult/=_nParticleNum;

  std::fill(_vWeight.begin(), _vWeight.end(), 1.0/_vWeight.size());
}

void CJPDARectangleFilter::Process(BoostVec& rvX, const BoostVec& rvXBefore) {

  rvX = rvXBefore;

  //現在の速度
  double dV = FastMath::fast_hypot(rvX(2), rvX(3));
  CAngle dR(FastMath::table_atan2(rvX(3), rvX(2)));

  dV+=_dVNoise1*Random();
  dR+=_dWNoise1*Random();

  rvX(2) = dV*FastMath::table_cos(dR.get());
  rvX(3) = dV*FastMath::table_sin(dR.get());

  rvX(0) += rvX(2)*_dTimeStep;
  rvX(1) += rvX(3)*_dTimeStep;
  rvX(0) += Random()*_vSystemNoise(0);
  rvX(1) += Random()*_vSystemNoise(1);

  rvX(4) += Random()*_vSystemNoise(4);
  rvX(5) += Random()*_vSystemNoise(5);
  rvX(4) = max(_dMinLength, min(_dMaxLength, rvX(4)));
  rvX(5) = max(_dMinWidth, min(_dMaxWidth, rvX(5)));

}

struct SSquare4 {

  double _dX1; double _dY1; //L1, L3の交点
  double _dX2; double _dY2; //L2, L3の交点
  double _dX3; double _dY3; //L1, L4の交点
  double _dX4; double _dY4; //L2, L4の交点

  std::vector< std::pair<double, double> > _vdPoints;

  SSquare4(const BoostVec &rX) {

    double dAngle = FastMath::table_atan2(rX(3), rX(2));
    double dLen1 = FastMath::fast_hypot(rX(4)/2, rX(5)/2);

    double dAngle1 = dAngle + FastMath::table_atan2(rX(5)/2, rX(4)/2);
    double dC1 = cos(dAngle1);
    double dS1 = sin(dAngle1);
    _dX1 = rX(0) + dLen1*dC1;
    _dY1 = rX(1) + dLen1*dS1;

    double dAngle2 = dAngle + (M_PI-FastMath::table_atan2(rX(5)/2, rX(4)/2));
    double dC2 = cos(dAngle2);
    double dS2 = sin(dAngle2);

    _dX2 = rX(0) + dLen1*dC2;
    _dY2 = rX(1) + dLen1*dS2;

    _dX3 = rX(0) - dLen1*dC1;
    _dY3 = rX(1) - dLen1*dS1;

    _dX4 = rX(0) - dLen1*dC2;
    _dY4 = rX(1) - dLen1*dS2;

    _vdPoints.push_back(make_pair(_dX1, _dY1));
    _vdPoints.push_back(make_pair(_dX2, _dY2));
    _vdPoints.push_back(make_pair(_dX3, _dY3));
    _vdPoints.push_back(make_pair(_dX4, _dY4));
  }

  void GetPFPoints(boost::shared_ptr<const CCascadedCoords> pLRFCo, const SLRFProperty &rProp, //in
    vector<std::pair<int, double> > &vdDists) //out index/dist
  {

    double dcx = pLRFCo->GetPos()(0);
    double dcy = pLRFCo->GetPos()(1);
    double dYaw = pLRFCo->GetYaw();
    vector<CAngle> vdAngles(_vdPoints.size());
    for (size_t i=0; i<_vdPoints.size(); ++i) {
      vdAngles[i] = FastMath::table_atan2(_vdPoints[i].second-dcy, _vdPoints[i].first-dcx) - dYaw;
    }

    for (size_t i=0; i<_vdPoints.size(); ++i) {
      size_t i2 = i+1; if (i2 == _vdPoints.size()) i2 = 0;
      if (vdAngles[i] > vdAngles[i2]) {

        int nIndex2 = (int)ceil(rProp.AngleToContinuousIndex(vdAngles[i2]));
        CAngle dFirst = rProp.IndexToAngleWithoutException(nIndex2); //Laserに揃える
        int nIndex1 = (int)ceil(rProp.AngleToContinuousIndex(vdAngles[i]));
        CAngle dLast = rProp.IndexToAngleWithoutException(nIndex1);

        double dD1 = FastMath::fast_hypot(_vdPoints[i2].first - dcx, _vdPoints[i2].second - dcy);
        double dD2 = FastMath::fast_hypot(_vdPoints[i].first - dcx, _vdPoints[i].second - dcy);
        double dD3 = FastMath::fast_hypot(_vdPoints[i].first - _vdPoints[i2].first,
                                          _vdPoints[i].second - _vdPoints[i2].second);

        CAngle dR3 = vdAngles[i]-vdAngles[i2];
        double dSinR2 = dD2/dD3*sin(dR3);
        double dR2 = asin(dSinR2);
        if (dD2*dD2 > (dD1*dD1+dD3*dD3)) { //鈍角
          dR2 = M_PI-dR2;
        }
        for (CAngle dAngle = dFirst; dAngle < dLast; dAngle += rProp._dReso) {
          auto nID = rProp.AngleToIndexWithoutException(dAngle);
          if (nID < rProp._nElemNum) {
            CAngle dR = dAngle - vdAngles[i2];
            double dD = dD1 / (sin(dR+dR2)) * dSinR2;
            vdDists.push_back(make_pair((int)nID, dD));
          }
        }
      }
    }
  }
};

//temp sort
bool SortPair(const pair<int, double> &r1, const pair<int, double> &r2) {
  return r1.first < r2.first;
}

bool CJPDARectangleFilter::CalcParticleCorrelation(const SJPDAFilterData &rData, int nPFNum, std::vector<std::pair<int, double>> &rvCorrelations) {

  const auto &pLaserCo = rData._pLaserData->GetCo();
  const auto &vPosGlobal = _avPredict.at(nPFNum);
  const auto &rProp = rData._pLaserData->GetProperty();

  SSquare4 s2(vPosGlobal);
  //LRFが長方形内にあったらrvCorrelationsは空のはず
  rvCorrelations.clear();
  s2.GetPFPoints(pLaserCo, rProp, rvCorrelations);
  std::sort(rvCorrelations.begin(), rvCorrelations.end(), SortPair);

  if (rvCorrelations.empty())return false;
  else return true;
}


double CJPDARectangleFilter::GetNearestDistFromParticle(int nPFNo, double x, double y,  const boost::shared_ptr<const CCascadedCoords> &pLRFCo) {


  return 100;

}