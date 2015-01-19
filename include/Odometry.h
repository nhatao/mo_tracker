#pragma once

#include "Coordinates.h"
class COdometry
{
public:

  COdometry() {}
  virtual ~COdometry(void) {}

  virtual bool Measure() = 0;
  virtual void Reset() = 0;
  virtual void GetWorldCoords(CCoordinates2D &rResultCo) const = 0;
  virtual void GetOneStepCoords(CCoordinates2D &rResultCo) const = 0;

  const CCoordinates2D &GetWorldCoords() const {
    GetWorldCoords(_temp1); return _temp1;
  };
  const CCoordinates2D &GetOneStepCoords() const {
    GetOneStepCoords(_temp2); return _temp2;
  };

  virtual bool HasSwing () const {return false;}
  virtual double GetRoll() const {return 0;}
  virtual double GetPitch() const {return 0;}
  virtual double GetVelX() const {return 0;}
  virtual double GetVelY() const {return 0;}
  virtual double GetVelR() const {return 0;}

private:

  mutable CCoordinates2D _temp1;
  mutable CCoordinates2D _temp2;

};