#pragma once

#include "CDigitizedSpace.h"

template <class T>
class CGridSpaceTemplate: public CDigitizedSpace<T, 2>
{
public:
  typedef CDigitizedSpace<T, 2> P;
  typedef CGridSpaceTemplate<T> C;

  CGridSpaceTemplate<T>() {};
  CGridSpaceTemplate<T>(double dXMax, double dXMin, double dYMax, double dYMin, double dXGridSize, double dYGridSize);
  virtual ~CGridSpaceTemplate<T>(void) {};
  void Resize(double dXMax, double dXMin, double dYMax, double dYMin, double dXGridSize, double dYGridSize);

  //グリッド中心の座標を返す
  double GridToPosX(int x) const {return (x*_dXGridSize)+_dXMin+_dXGridSize/2;}
  double GridToPosY(int y) const {return (y*_dYGridSize)+_dYMin+_dYGridSize/2;}
  //離散値の値を返す
  double GridToPosXD(double x) const {return (x*_dXGridSize)+_dXMin;}
  double GridToPosYD(double y) const {return (y*_dYGridSize)+_dYMin;}

  double GetXGridSize() const {return _dXGridSize;};
  double GetYGridSize() const {return _dYGridSize;};

  int GetXPos(double dX) const {
    return (int)floor((dX-_dXMin)/_dXGridSize);
  }
  int GetYPos(double dY) const {
    return (int)floor((dY-_dYMin)/_dYGridSize);
  }
  double GetXPosD(double dX) const {
    return (dX-_dXMin)/_dXGridSize;
  }
  double GetYPosD(double dY) const {
    return (dY-_dYMin)/_dYGridSize;
  }

  bool IsInBound(int x, int y) const {
    return ( (x>=0) && (x < GetXGridNum()) && (y>=0) && (y < GetYGridNum()) );
  }
  bool IsInBoundCarte(double x, double y) const {
    return IsInBound(GetXPos(x), GetYPos(y));
  }
  //絶対座標上での座標でグリッドを指定
  bool ApplyCarteCoords(double x, double y, const typename P::UnitFunc& Func) {
    return ApplyGridCoords(GetXPos(x), GetYPos(y), Func);
  }
  bool ApplyCarteCoords(const BoostVec& vPos, const typename P::UnitFunc& Func) {
    return ApplyGridCoords(GetXPos(vPos(0)), GetYPos(vPos(1)), Func);
  }
  //グリッド座標系でグリッドを指定
  bool ApplyGridCoords(int x1, int y1, const typename P::UnitFunc& Func) {
    if (IsInBound(x1,y1)) {
      ApplySpecific(GetArrayPos(x1,y1), Func);
      return true;
    }
    else return false;
  }
  int GetArrayPos(int x, int y) const {
    return y+x*P::_anSpaceSize[1];
  }
  int GetArrayPosWithCheck(int x, int y) const {
    if (IsInBound(x,y)) return y+x*P::_anSpaceSize[1];
    return -1;
  }
  T* GetOneObject(int x, int y) {
    int nPos = GetArrayPosWithCheck(x,y);
    if (nPos>=0) return P::GetObjects()+nPos;
    else return NULL;
  }
  const T* GetOneObject(int x, int y) const {
    int nPos = GetArrayPosWithCheck(x,y);
    if (nPos>=0) return P::GetObjects()+nPos;
    else return NULL;
  }
  T* GetOneObjectInCarteCoords(double x, double y) {
    return GetOneObject(GetXPos(x),GetYPos(y)); 
  }
  const T* GetOneObjectInCarteCoords(double x, double y) const {
    return GetOneObject(GetXPos(x),GetYPos(y)); 
  }


  int GetXGridNum() const {return P::_anSpaceSize[0];};
  int GetYGridNum() const {return P::_anSpaceSize[1];};

  double GetXMin() const {return _dXMin;}
  double GetYMin() const {return _dYMin;}
  double GetXMax() const {return _dXMax;}
  double GetYMax() const {return _dYMax;}

private:

  /*
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version){
    ar & _dXMax & _dYMax & _dXMin & _dYMin & _dXGridSize & _dYGridSize;
    ar & boost::serialization::base_object<CDigitizedSpace<T, 2> >(*this);
  }
  friend class boost::serialization::access;
  */

  double _dXMax;
  double _dYMax;
  double _dXMin;
  double _dYMin;

  double _dXGridSize;
  double _dYGridSize; 

};

template <class T>
void RegisterNeighborsGrid(T &rUnit, const CGridSpaceTemplate<T>* pGrid, int pNeighbors[][2], int nSize) {

  int i = rUnit.aPos[0];
  int j = rUnit.aPos[1];
  rUnit.vnNeighbors->clear();
//  int cpos = pGrid->GetArrayPos(i,j);
  for(int n=0; n< nSize; ++n) {
    int pos = pGrid->GetArrayPosWithCheck(i+(pNeighbors)[n][0],j+(pNeighbors)[n][1]);
    if (pos>0) {
      rUnit.vnNeighbors->push_back(pos);
    }
  }
}

template <class T>
void RegisterNeighbors4(T &rUnit, const CGridSpaceTemplate<T>* pGrid) {
  int pNeighbors[][2] = {{1,0},{0,1},{-1,0},{0,-1}};
  RegisterNeighborsGrid(rUnit, pGrid, pNeighbors, 4);
}

template <class T>
void RegisterNeighbors8(T &rUnit, const CGridSpaceTemplate<T>* pGrid) {
  int pNeighbors[][2] = {{1,0},{0,1},{-1,0},{0,-1},{1,1},{-1,1},{-1,-1},{1,-1}};
  RegisterNeighborsGrid(rUnit, pGrid, pNeighbors, 8);
}



template <class T>
CGridSpaceTemplate<T>::CGridSpaceTemplate(double dXMax, double dXMin, double dYMax, double dYMin, double dXGridSize, double dYGridSize) {

  Resize(dXMax, dXMin, dYMax, dYMin, dXGridSize, dYGridSize);
}


template <class T>
void CGridSpaceTemplate<T>::Resize(double dXMax, double dXMin, double dYMax, double dYMin, double dXGridSize, double dYGridSize) {

  _dXMax       = dXMax;
  _dXMin       = dXMin;
  _dYMax       = dYMax;
  _dYMin       = dYMin;
  _dXGridSize  = dXGridSize;
  _dYGridSize  = dYGridSize;

  int anSize[2];
  anSize[0] = (int)(ceil((_dXMax-_dXMin)/_dXGridSize));
  anSize[1] = (int)(ceil((_dYMax-_dYMin)/_dYGridSize));
  P::Init(anSize);

  int nx = GetXGridNum();
  int ny = GetYGridNum();

  for (int i=0; i<nx; ++i) {
    for (int j=0; j<ny; ++j) {        
      int nNum = GetArrayPos(i,j);
      P::_aUnits[nNum].aPos[0] = i;
      P::_aUnits[nNum].aPos[1] = j;
    }
  }
  for (int i=0; i<P::_nTotalUnitNum; ++i) {
    RegisterNeighbors4(P::_aUnits[i], this);
  }
}


class CCoordinates3D;
struct location;

struct SUnit: public SSimpleUnit {
  double dLikelihood;
  unsigned char dRAverage;
  unsigned char dGAverage;
  unsigned char dBAverage;
  bool bChanged;
  static double dLikelihoodThresh;

  SUnit() : SSimpleUnit(){
    dLikelihood=0.5;
    dRAverage=128; dGAverage=128; dBAverage=128;
    bChanged = false;
  }
  virtual ~SUnit() {}

  void Dump(std::ostream &rOS) const{
    oswrite(rOS, nVotedNum);
    oswrite(rOS, nLabel);
    oswrite(rOS, dLikelihood);
    oswrite(rOS, dRAverage);
    oswrite(rOS, dGAverage);
    oswrite(rOS, dBAverage);
  }

  void Load(std::istream &rIS) {
    isread(rIS, nVotedNum);
    isread(rIS, nLabel);
    isread(rIS, dLikelihood);
    isread(rIS, dRAverage);
    isread(rIS, dGAverage);
    isread(rIS, dBAverage);
    bChanged = true;
  }

  virtual bool IsEnable() const{
    return (dLikelihood > dLikelihoodThresh);
  }

private:
};

typedef CGridSpaceTemplate<SUnit> CGridSpace;
