#pragma once

#include "LRFSensor.h"
#include "PolygonalRegion.h"
#include "ColorCout.h"
#include "MOTracker.h"
class CTrackInitialization
{
public:

  CTrackInitialization(void) {} ;
  virtual ~CTrackInitialization(void) {};
  virtual void Update(const LaserDataBuffer &Log, std::vector<int> &vnExtracted)=0;
  virtual bool IsOutOfRange(const BoostVec &rv) = 0;
  virtual void GetAreaInfo(double &rdXMin, double &rdXMax, double &rdYMin, double &rdYMax) const = 0;
  virtual void GetTargetRegion(CPolygonalRegion &rRegion) const {
    double dXMin, dYMin, dXMax, dYMax;
    GetAreaInfo(dXMin, dXMax, dYMin, dYMax);
    rRegion.SetRectangleRegion(dXMin, dYMin, dXMax, dYMax);
  }
  //virtual void IsMoving(const BoostVec &rv, const SLRFProperty& rProp, boost::shared_ptr<const CCascadedCoords> pCo) = 0;

protected:
};


class CTrackInitializationWithProperty : public CTrackInitialization {

public:

  CTrackInitializationWithProperty(void) {
    _bUpdateBackGround = true;
  }
  virtual ~CTrackInitializationWithProperty(void) {};

  void SetUpdateBackGround (bool b) {_bUpdateBackGround = b;}
  bool GetUpdateBackGround() const {return _bUpdateBackGround;}
  void SetInvalidRegion(const std::vector<std::pair<size_t, CPolygonalRegion> > &rvRegions) {
    _vInvalidRegions = rvRegions;
  }
  void SetValidRegion(const std::vector<std::pair<size_t, CPolygonalRegion> > &rvRegions) {
    _vValidRegions = rvRegions;
  }
  const std::vector<std::pair<size_t, CPolygonalRegion> > &GetInvalidRegion() const {return _vInvalidRegions;}
  const std::vector<std::pair<size_t, CPolygonalRegion> > &GetValidRegion() const {return _vValidRegions;}

  virtual void SaveBackGroundToFile(const std::string &rsFileName) const = 0;
  virtual void LoadBackGroundFromFile(const std::string &rsFileName) = 0;
  virtual void ResetBackGround() = 0;
  virtual void SetOdds(double dObsOdds, double dFreeOdds) = 0;
  virtual void SetStatusThr(double dObsThr, double dFreeThr) = 0;
  virtual void SetOddsMinMax(double dOddsMin, double dOddsMax) = 0;

protected:

  bool _bUpdateBackGround;
  std::vector<std::pair<size_t, CPolygonalRegion> > _vInvalidRegions;
  std::vector<std::pair<size_t, CPolygonalRegion> > _vValidRegions;
  bool IsPointValid(const BoostVec &rV, size_t nLRFNo) const;
};
