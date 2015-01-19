#pragma once

#include "CascadedCoords.h"
#include "MatrixUtils.h"

class CCascadedVec : public ICascadedObjects
{
public:

  static boost::shared_ptr<CCascadedVec> MakeCasVec(double x, double y, double z, boost::shared_ptr<const CCascadedCoords> pParent = CCascadedCoords::GetWorldAnchor());

  virtual ~CCascadedVec(void);
  const BoostVec &GetLocalVec() const;
  const BoostVec &GetWorldVec() const;

  double GetX() const { return GetWorldVec()(0);}
  double GetY() const { return GetWorldVec()(1);}
  double GetZ() const { return GetWorldVec()(2);}
  double GetLocalX() const { return GetLocalVec()(0);}
  double GetLocalY() const { return GetLocalVec()(1);}
  double GetLocalZ() const { return GetLocalVec()(2);}

  void SetWorld(double x, double y, double z);
  void SetLocal(double x, double y, double z);

  void SetParent(boost::shared_ptr<const CCascadedCoords> p) {
    _pParent = p;
  }
  boost::shared_ptr<const CCascadedCoords> GetParent() const {return _pParent;}
  virtual void NotifyUpdate() const {
    _bChanged = true;
  }

protected:

  CCascadedVec(double x, double y, double z, boost::shared_ptr<const CCascadedCoords> pParent);
  CCascadedVec(const CCascadedVec &rhs);
  CCascadedVec& operator= (const CCascadedVec &rhs);

  BoostVec _LocalVec;
  mutable BoostVec _WorldVec;
  boost::shared_ptr<const CCascadedCoords> _pParent;

  mutable bool _bChanged;
};

