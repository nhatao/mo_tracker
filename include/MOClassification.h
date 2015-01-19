#pragma once

#include "JPDATracker.h"

struct SClassificationResult {
  struct SMOStatus {
    double dProb;
    std::vector<int> vIDs;
//    BoostVec vInitVec;
  };
  std::vector<SMOStatus> vCars;
  std::vector<SMOStatus> vPersons;

  size_t GetPersonNum() const {return vPersons.size();}
  size_t GetCarNum() const {return vCars.size();}
};

class CMOClassification {
public:
  CMOClassification() {}
  virtual ~CMOClassification() {}
  virtual boost::shared_ptr<SClassificationResult> Proc(const std::vector<boost::shared_ptr<CJPDATracker::SPointCluster>> &rvpCluster) = 0;
  virtual void SetPerformCarDetection(bool b) = 0;
  virtual bool PerfomCarDetection() const = 0;

private:

};
