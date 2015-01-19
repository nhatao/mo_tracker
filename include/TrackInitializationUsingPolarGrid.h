#pragma once
#include "TrackInitialization.h"
#include <boost/thread.hpp>


class CTrackInitializationUsingPolarGrid :
  public CTrackInitializationWithProperty
{
public:
  //nStep=nにすると隣接するn個のLRF点をまとめる
  CTrackInitializationUsingPolarGrid(double dMaxLen, double dGridSize, size_t nLaserNum, size_t nStep=1);
  virtual ~CTrackInitializationUsingPolarGrid(void);
  virtual bool IsOutOfRange(const BoostVec &rv);
  virtual void GetAreaInfo(double &rdXMin, double &rdXMax, double &rdYMin, double &rdYMax) const;
  void SetParams(double dObsThr, double dFreeThr, double dLRFObsOdds, double dLRFFreeOdds, double dLRFOddsMax, double dLRFOddsMin);

  virtual void SaveBackGroundToFile(const std::string &rsFileName) const;
  virtual void LoadBackGroundFromFile(const std::string &rsFileName);
  virtual void ResetBackGround();

  virtual void SetOdds(double dObsOdds, double dFreeOdds);
  virtual void SetStatusThr(double dObsThr, double dFreeThr);
  virtual void SetOddsMinMax(double dOddsMin, double dOddsMax);
  double GetOddsMin() const {return _dLRFOddsMin;}
  double GetOddsMax() const {return _dLRFOddsMax;}
  double GetObsThr() const {return _dObsThr;}
  double GetFreeThr() const {return _dFreeThr;}
  double GetObsOdds() const {return _dLRFObsOdds;}
  double GetFreeOdds() const {return _dLRFFreeOdds;}

  size_t GetArrayXLen() const {return _nArrayX;}
  size_t GetArrayYLen() const {return _nArrayY;}
  const std::vector<double> &GetArray() const {return _aArray;}


  struct SCurrentStatus {
    unsigned int nLaserPos;
    double dLen;
    unsigned int nCellStatus;
    unsigned int nBeforeOcc;
  };
  const std::vector<SCurrentStatus> & GetCurrentStatus() const {
    return _vCurrentStatus;
  }
  const std::vector<SCurrentStatus> & GetCurrentStatusDebug() const {
    return _vCurrentStatusDebug;
  }
  //ビューワから呼び出す マルチスレッド対応
  void CopyArray(std::vector<double> &rvCopy) const {
    _ArrayMutex.lock();
    rvCopy = _aArray;
    _ArrayMutex.unlock();
  }
  void CopyStatus(std::vector<SCurrentStatus> &rStatus) const{
    _StatusMutex.lock();
    rStatus = _vCurrentStatus;
    _StatusMutex.unlock();
  }
  void CopyStatusDebug(std::vector<SCurrentStatus> &rStatus) const{
    _StatusMutex.lock();
    rStatus = _vCurrentStatusDebug;
    _StatusMutex.unlock();
  }
  unsigned int GetArrayPos(unsigned int nPosX, unsigned int nPosY) const {
    return _nArrayY*nPosX + nPosY;
  }
  unsigned int GetLineFirstPos(unsigned int nPosX) const {
    return _nArrayY*nPosX;
  }
  std::pair<unsigned int, unsigned int> GetArrayXY(size_t nLRFPos, double dDist) const;

  virtual void Update(const LaserDataBuffer &Log, std::vector<int> &vnExtracted);
  virtual bool IsMoving(const BoostVec &rv, const SLRFProperty& rProp, boost::shared_ptr<const CCascadedCoords> pCo);

  void UpdateGrid (const std::vector<double>& rArray);
  void SetMinDistThr(double d) {
    _dMinDistThr = d;
  }
  double GetMinDistThr() const {return _dMinDistThr;}
protected:

  double _dMaxLen;
  double _dGridSize;
  size_t _nLaserNum;
  size_t _nStep;
  size_t _nArrayX; //角度
  size_t _nArrayY; //距離
  double _dMinDistThr;

  std::vector<double> _aArray;
  double _dObsThr;
  double _dFreeThr;
  double _dLRFObsOdds;
  double _dLRFFreeOdds;
  double _dLRFOddsMax;
  double _dLRFOddsMin;

  void ReconstructArray();

  std::pair<unsigned int, unsigned int> GetPointStatus(size_t nLRFPos, double dDist) const;

  bool IsMovingRegion(unsigned int nStatus, unsigned int nOccupiedStatus) const;

  bool IsObstacle(double dVal) {
    return (dVal > _dObsThr);
  }
  bool IsFree(double dVal) {
    return (dVal < _dFreeThr);
  }
  bool IsUnknown(double dVal) {
    return ((_dFreeThr <= dVal) && (dVal <= _dObsThr));
  }

  std::vector<SCurrentStatus> _vCurrentStatus;
  std::vector<SCurrentStatus> _vCurrentStatusDebug;
  mutable boost::mutex _StatusMutex;
  mutable boost::mutex _ArrayMutex;

  mutable BoostVec _vGlobal;
  bool IsPointValid2(double dLen, const CAngle& dDeg, const CCoordinates2D &rLRFCo, size_t nLRFNo) const;

};

