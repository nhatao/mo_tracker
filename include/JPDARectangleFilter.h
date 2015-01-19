#pragma once
#include "JPDAFilter.h"
#include "PolygonalRegion.h"
/*
長方形形状で自動車を追跡するFilter
6次元: x, y, vx, vy, length, width
lengthの方向が速度ベクトルと一致
*/

struct SBBInfo {
  std::vector<BoostVec> vBBPos; //四隅座標
  CAngle dBBAngle; //傾き
  double dLLen;    //長辺の長さ
  double dSLen;    //短辺の長さ
  double dResidual;//残差
  double dResidualFrontRatio; //陽線に近い点の割合 点が2個以下なら1とする
};

class CJPDARectangleFilter :
  public CJPDAFilter
{
public:

  CJPDARectangleFilter(size_t nParticleNum);
  virtual ~CJPDARectangleFilter() {}
  void ResetParticles(const CCoordinates3D &rLRFCo, const BoostVec &rInitPos);
  void ResetParticles(const SJPDAFilterData& rData, const std::vector<int> &rvIDs, const SBBInfo &rBBInfo);
  virtual void SetTimeStep(double dTimeStep);

  //dEnlargeLen: 相関判定などのため矩形サイズを広げたいときに使う 
  void GetResultPolygonal(CPolygonalRegion &r, double dEnlargeLen=0);
  void GetResultPoints(std::vector<BoostVec> &rvPoints, double dEnlargeLen=0);

protected:
  virtual void Process(BoostVec& rvX, const BoostVec& rvXBefore);

//  virtual void CalcRange(const SJPDAFilterData &rData, int nPFNum, int &rnMin, int &rnMax, std::vector<double> &rvnDistance);
  virtual bool CalcParticleCorrelation(const SJPDAFilterData &rData, int nPFNum, std::vector<std::pair<int, double>> &rvCorrelations);
  double _dXN;
  double _dRVN;
  double _dVN;

  double _dL1N;
  double _dL2N;


  double _dMinWidth;
  double _dMaxWidth;
  double _dMinLength;
  double _dMaxLength;

  double _dVNoise1;
  double _dWNoise1;

  virtual double GetNearestDistFromParticle(int nPFNo, double x, double y, const boost::shared_ptr<const CCascadedCoords> &pLRFCo);
};
