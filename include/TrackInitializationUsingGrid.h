#pragma once
#include "TrackInitialization.h"
#include <boost/shared_ptr.hpp>
#include "GridSpace.h"

class CTrackInitializationUsingGrid :
  public CTrackInitializationWithProperty
{
public:
  CTrackInitializationUsingGrid();
  virtual ~CTrackInitializationUsingGrid(void);

  virtual void Update(const LaserDataBuffer &Log, std::vector<int> &vnExtracted);

  virtual bool IsOutOfRange(const BoostVec &rv);

  void SetDistThresh(double d) {_dDistThresh=d;}
  double GetDistThresh() const {return _dDistThresh;}

  virtual void GetAreaInfo(double &rdXMin, double &rdXMax, double &rdYMin, double &rdYMax) const;

  virtual void LoadBackGroundFromFile(const std::string &rsFileName);
  virtual void SaveBackGroundToFile(const std::string &rsFileName) const;
  virtual void ResetBackGround();
  virtual void SetOdds(double dObsOdds, double dFreeOdds);
  virtual void SetStatusThr(double dObsThr, double dFreeThr);
  virtual void SetOddsMinMax(double dOddsMin, double dOddsMax);

  void Initialize(double dXMax, double dXMin, double dYMax, double dYMin, double dXYGridSize);
  void Initialize(const std::string &rsFileName);

  void SetFreeThrLRF(double d) {_dFreeThrLRF = d;}
  void SetObstacleThrLRF(double d) {_dObstacleThrLRF= d;}

  int GetXGridNum() const {return _nXGridNum;}
  int GetYGridNum() const {return _nYGridNum;}
  boost::shared_ptr<const CGridSpace> GetGrid() const {return _pGrid;}

  double GetFreeThr() const {return _dFreeThrLRF;}
  double GetObstacleThr() const {return _dObstacleThrLRF;}

protected:

  boost::shared_ptr<CGridSpace> _pGrid;

  double _dMaxX;
  double _dMinX;
  double _dMaxY;
  double _dMinY;
  double _dDistThresh;

  bool IsGridFree(const BoostVec &rV);

  double _dLRFOddsMin;
  double _dLRFOddsMax;
  double _dFreeThrLRF;
  double _dObstacleThrLRF;

  double _dLRFObsOdds;
  double _dLRFFreeOdds;

  int _nXGridNum;
  int _nYGridNum;

  void SetLRFData2(const std::vector<std::vector<double> > &vLaserData, const std::vector<CCoordinates2D> &rvCoords,
                   int nLaserNum, CAngle dDegBegin, CAngle dReso, double dLaserMaxLen);



};