#pragma once
#include "JPDAFilter.h"

/*
楕円形状で歩行者を追跡するFilter
6次元：x, y, vx, vy, r1, r2
*/

class CJPDAEllipseFilter : public CJPDAFilter
{
public:

  CJPDAEllipseFilter(size_t nParticleNum);
  virtual ~CJPDAEllipseFilter() {}
  void ResetParticles(const SJPDAFilterData& rData, const std::vector<int> &rvIDs);
  virtual void SetTimeStep(double dTimeStep);

  //0以下の値を入れると現時点でのパラメータを維持
  void SetSizeValues(double dR1Ave, double dR1StdDev, double dR1Max, double dR1Min,
    double dR2Ave, double dR2StdDev, double dR2Max, double dR2Min);
  void SetNoiseValues(double dXN, double dRadN, double dVN1, double dRVN1, double dVN2, double dRVN2, double dLowVelThr, double dRVNLowVel);


protected:
  virtual void Process(BoostVec& rvX, const BoostVec& rvXBefore);
  virtual bool CalcParticleCorrelation(const SJPDAFilterData &rData, int nPFNum, std::vector<std::pair<int, double>> &rvCorrelations);
  //パーティクルまでの最短距離を返す
  virtual double GetNearestDistFromParticle(int nPFNo, double x, double y, const boost::shared_ptr<const CCascadedCoords> &pLRFCo);

  //GetNearestDistFromParticleで使う．
  struct SParticleEdgeInfo {
    SParticleEdgeInfo() {
      Init();
    }
    void Init() {
      _dMatchRangeMin = 0;
      _dMatchRangeMax = 0;
      _dLeftEdgeX = 0;
      _dLeftEdgeY = 0;
      _dRightEdgeX = 0;
      _dRightEdgeY = 0;
    }
    //この範囲内に挟まれていたら手前側，それ以外は奥側
    CAngle _dMatchRangeMin;
    CAngle _dMatchRangeMax;
    //奥側にある点との最短距離をここに格納
    double _dLeftEdgeX;
    double _dLeftEdgeY;
    double _dRightEdgeX;
    double _dRightEdgeY;
  };
  std::vector<SParticleEdgeInfo> _vParticleEdgeInfo;


  double _dR1Ave;
  double _dR1StdDev;
  double _dR1Min;
  double _dR1Max;
  double _dR2Ave;
  double _dR2StdDev;
  double _dR2Min;
  double _dR2Max;

  //最後に足す位置と速度のノイズ
  double _dXN;
  double _dRadN;
  //直進
  double _dVN1;
  double _dRVN1;
  //旋回
  double _dVN2;
  double _dRVN2;
  //低速時
  double _dLowVelThr;
  double _dRVNLowVel;

  int _nCnt;

};
