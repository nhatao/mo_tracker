#pragma once
#include "TrackInitialization.h"

//与えられたLRFデータのうち、Min/Maxの範囲内全部を候補として抽出するInitializer
//ROSで前処理を行っている前提
class CTrackInitializationPassThrough :
  public CTrackInitializationWithProperty
{
public:
  CTrackInitializationPassThrough(void) {}
  virtual ~CTrackInitializationPassThrough(void) {}

  virtual bool IsOutOfRange(const BoostVec &rv) {return false;}
  virtual void GetAreaInfo(double &rdXMin, double &rdXMax, double &rdYMin, double &rdYMax) const {
    rdXMin = -DBL_MAX;
    rdXMax = DBL_MAX;
    rdYMin = -DBL_MAX;
    rdYMax = DBL_MAX;
  }
  virtual void Update(const LaserDataBuffer &rLog, std::vector<int> &vnExtracted);

  //プログラムの都合上インタフェースだけ作る
  virtual void SaveBackGroundToFile(const std::string &rsFileName) const {}
  virtual void LoadBackGroundFromFile(const std::string &rsFileName) {}
  virtual void ResetBackGround() {}
  virtual void SetOdds(double dObsOdds, double dFreeOdds) {}
  virtual void SetStatusThr(double dObsThr, double dFreeThr) {}
  virtual void SetOddsMinMax(double dOddsMin, double dOddsMax) {}
};

