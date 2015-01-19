#pragma once
#include "MOClassification.h"

class CSimpleClassification :
  public CMOClassification
{
public:
  CSimpleClassification(void);
  virtual ~CSimpleClassification(void);

  virtual boost::shared_ptr<SClassificationResult> Proc(const std::vector<boost::shared_ptr<CJPDATracker::SPointCluster>> &rvpCluster);
  
  virtual void SetPerformCarDetection(bool) {} //Cannot detect cars
  virtual bool PerfomCarDetection() const {return false;}

  struct SParams {
    double dTotalLenAverage;
    double dTotalLenSigma;
    double dDistBetweenEdgeAverage;
    double dDistBetweenEdgeSigma;
  };

  //vector上の位置が歩行者の数と対応
  void SetParams(const std::vector<SParams> &rvParams) {
    _vParams = rvParams;
  }
  //dTotalLenがしきい値以下の時FalsePositiveとみなす
  void SetTotalLenMinThreshMultiplier(double d) {
    if (d > 0) {
      _dTotalLenMinThreshMultiplier = d;
    }
  }
  void SetTotalLenMaxThreshMultiplier(double d) {
    if (d > 0) {
      _dTotalLenMaxThreshMultiplier = d;
    }
  }
  void SetDistBetweenEdgeMinThreshMultiplier(double d) {
    if (d > 0) {
      _dDistBetweenEdgeMinThreshMultiplier = d;
    }
  }
  void SetDistBetweenEdgeMaxThreshMultiplier(double d) {
    if (d > 0) {
      _dDistBetweenEdgeMaxThreshMultiplier = d;
    }
  }

protected:
  std::vector<SParams> _vParams;
  double _dTotalLenMinThreshMultiplier;
  double _dTotalLenMaxThreshMultiplier;
  double _dDistBetweenEdgeMinThreshMultiplier;
  double _dDistBetweenEdgeMaxThreshMultiplier;


};
