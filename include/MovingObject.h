#pragma once

#include "MatrixFuncs.h"
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/noncopyable.hpp>

//これを派生させて必要なデータを追加
struct SMovingObjectStatus : boost::noncopyable{
  SMovingObjectStatus() {
    _vPos.resize(3); _vPos(0) = 0; _vPos(1) = 0; _vPos(2) = 0;
    _vVel.resize(3); _vVel(0) = 0; _vVel(1) = 0; _vVel(2) = 0;
    _dExistenceRate = 1;
  }
  virtual ~SMovingObjectStatus() {};
  BoostVec _vPos;
  BoostVec _vVel;
  double _dExistenceRate;
  std::string _sStatus;

  virtual boost::shared_ptr<SMovingObjectStatus> Clone() const {

    auto* p = new SMovingObjectStatus();
    p->_vPos = _vPos;
    p->_vVel = _vVel;
    p->_dExistenceRate = _dExistenceRate;
    p->_sStatus = _sStatus;
    return boost::shared_ptr<SMovingObjectStatus>(p);
  }
};

struct SCylinderMovingObjectStatus : public SMovingObjectStatus  {
  SCylinderMovingObjectStatus() :  SMovingObjectStatus() {
    _dRadius = 0;
    _dMinHeight = 0;
    _dMaxHeight = 0;
  }
  ~SCylinderMovingObjectStatus() {
  }

  double _dRadius;
  double _dMinHeight;
  double _dMaxHeight;


  virtual boost::shared_ptr<SMovingObjectStatus> Clone() const {
    auto* p = new SCylinderMovingObjectStatus();
    p->_vPos = _vPos;
    p->_vVel = _vVel;
    p->_dExistenceRate = _dExistenceRate;
    p->_dRadius = _dRadius;
    p->_dMinHeight = _dMinHeight;
    p->_dMaxHeight = _dMaxHeight;
    p->_sStatus = _sStatus;
    return boost::shared_ptr<SMovingObjectStatus>(p);
  }
};

//向きはvelの向きと同じ
struct SEllipseCylinderMovingObjectStatus : public SMovingObjectStatus  {
  SEllipseCylinderMovingObjectStatus() :  SMovingObjectStatus() {
    _dR1 = 0;
    _dR2 = 0;
    _dMinHeight = 0;
    _dMaxHeight = 0;
  }
  ~SEllipseCylinderMovingObjectStatus() {
  }

  double _dR1;
  double _dR2;
  double _dMinHeight;
  double _dMaxHeight;

  virtual boost::shared_ptr<SMovingObjectStatus> Clone() const {
    auto* p = new SEllipseCylinderMovingObjectStatus();
    p->_vPos = _vPos;
    p->_vVel = _vVel;
    p->_dExistenceRate = _dExistenceRate;
    p->_dR1 = _dR1;
    p->_dR2 = _dR2;
    p->_dMinHeight = _dMinHeight;
    p->_dMaxHeight = _dMaxHeight;
    p->_sStatus = _sStatus;
    return boost::shared_ptr<SMovingObjectStatus>(p);
  }
};

//向きはvelの向きと同じ
struct SCuboidMovingObjectStatus : public SMovingObjectStatus  {
  SCuboidMovingObjectStatus() :  SMovingObjectStatus() {
    _dLength = 0;
    _dWidth = 0;
    _dMinHeight = 0;
    _dMaxHeight = 0;
  }
  ~SCuboidMovingObjectStatus() {
  }

  double _dLength;
  double _dWidth;
  double _dMinHeight;
  double _dMaxHeight;

  virtual boost::shared_ptr<SMovingObjectStatus> Clone() const {
    auto* p = new SCuboidMovingObjectStatus();
    p->_vPos = _vPos;
    p->_vVel = _vVel;
    p->_dExistenceRate = _dExistenceRate;
    p->_dLength = _dLength;
    p->_dWidth = _dWidth;
    p->_dMinHeight = _dMinHeight;
    p->_dMaxHeight = _dMaxHeight;
    p->_sStatus = _sStatus;
    return boost::shared_ptr<SMovingObjectStatus>(p);
  }
};

#include <algorithm>

class CMovingObject : boost::noncopyable {
public:


  CMovingObject(const std::string &sType, int nID) {
    _sType = sType;
    _nID = nID;
    _vpStatusLog.set_capacity(s_nHistoryMaxNum);
  }
  ~CMovingObject() {
  }

  void AddCurrentStatus(boost::shared_ptr<const SMovingObjectStatus> pStatus, int nFrame) {
    _vpStatusLog.push_back(pStatus);
    _nLatestFrame = nFrame;
  }

  boost::shared_ptr<const SMovingObjectStatus> GetStatus(int nFrame=-1) const {
    if (_vpStatusLog.empty()) {
      return boost::shared_ptr<const SMovingObjectStatus>();
    }
    int nTargetPos = (int)(_vpStatusLog.size()-1); //nFrameが負値なら最新の結果
    if (nFrame >= 0) {
      nTargetPos -= (_nLatestFrame-nFrame);
    }
    //2個めの条件はありえないが念のため
    if ((nTargetPos < 0) || (nTargetPos >= (int)_vpStatusLog.size())) {
      return boost::shared_ptr<const SMovingObjectStatus>();
    }
    return _vpStatusLog[nTargetPos];
  }

  int GetID() const {return _nID;}
  const std::string &GetType() const {return _sType;}

//  const std::vector<boost::shared_ptr<const SMovingObjectStatus> > & GetStatusLog() const {return _vpStatusLog;}
  const boost::circular_buffer<boost::shared_ptr<const SMovingObjectStatus> > & GetStatusLog() const {return _vpStatusLog;}


  virtual boost::shared_ptr<CMovingObject> Clone() const {
    CMovingObject* p = new CMovingObject(_sType, _nID);
    p->_nLatestFrame = _nLatestFrame;
    for (auto it=_vpStatusLog.begin(); it!=_vpStatusLog.end(); ++it) {
      p->_vpStatusLog.push_back((*it)->Clone());
    }
    return boost::shared_ptr<CMovingObject>(p);
  }
  static void SetHistoryMaxNum(int n) {
    s_nHistoryMaxNum = n;
  }

protected:

  static int s_nHistoryMaxNum;

  int _nID;
  std::string _sType;
//  std::vector<boost::shared_ptr<const SMovingObjectStatus> > _vpStatusLog;
  boost::circular_buffer<boost::shared_ptr<const SMovingObjectStatus> > _vpStatusLog;
  int _nLatestFrame;
};


//Trackerはこれを派生させたクラスを結果として出力する
struct STrackerResult : boost::noncopyable{

  virtual ~STrackerResult() {};

  double _dCurrentTime;
  int _nFrame;
  std::vector<boost::shared_ptr<CMovingObject> > _vpObjects;
  double _dProcTime; //処理時間
  double _dStepTime; //前のフレームとの時間差

};
