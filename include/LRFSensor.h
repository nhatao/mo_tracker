#pragma once
#include "NHSensorManager.h"
#include "URGSource.h"

class CLRFSensor;
class CLRFStatus : public INHSensorStatus
{
public:
  CLRFStatus(int nPointNum, double dFirstDeg, double dReso, const INHSensor* pParent);
  CLRFStatus(const SLRFProperty &rProperty, const INHSensor* pParent);
  CLRFStatus(const INHSensor* pParent);
  virtual ~CLRFStatus();
  CLRFStatus(const CLRFStatus& rhs) : INHSensorStatus(_pParent) {
    this->operator=(rhs);
  }
  CLRFStatus& operator=(const CLRFStatus& rhs);
  const std::vector<double>& GetBuf() const {return _vBuf;}
  int GetPointNum() const {return _Property._nElemNum;}
  double GetFirstDeg() const {return _Property._dFirstAngle.get_deg();}
  double GetResolution() const {return _Property._dReso.get_deg();}
  const SLRFProperty &GetProperty() const {return _Property;}

  friend class CLRFSensor;
protected:

  virtual void WriteToLogImpl(std::ostream &out, int nFrameNum);
  virtual void ReadFromLogImpl(std::istream &in);
  std::vector<double> _vBuf;

  SLRFProperty _Property;
};

class CBlackURGStatus : public CLRFStatus
{
public:
  CBlackURGStatus(const INHSensor* pParent) : CLRFStatus(440, -(219*360.0/552), 360.0/552.0, pParent) {}
  ~CBlackURGStatus() {};
};

class CTopURGStatus : public CLRFStatus
{
public:
  CTopURGStatus(const INHSensor* pParent) : CLRFStatus(1081, -135, 0.25, pParent) {}
  ~CTopURGStatus() {};
};

class CLRFSensor :
  public INHSensor
{
public:

  CLRFSensor(void);
  CLRFSensor(const std::string& rsName);
  virtual ~CLRFSensor(void);

  void SetLRF(boost::shared_ptr<CLRFSource> pLRF);
  const boost::shared_ptr<CLRFSource> GetLRF() const {return _pLRF;}

  virtual void Init();
  virtual bool Measure();
  virtual unsigned int GetMinUpdateTime() const;
  virtual boost::shared_ptr<INHSensorStatus> MakeSensorStatus() const;

  const SLRFProperty& GetProperty() const {return _Property;}

protected:

  virtual bool UpdateStatusImpl(INHSensorStatus *pStatus) const;
  boost::shared_ptr<CLRFSource> _pLRF;
  bool _bHasHardwareClock;

  SLRFProperty _Property;

};
