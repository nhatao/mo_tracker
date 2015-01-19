#pragma once

#include <vector>
#include <opencv/cv.h>
#include <boost/noncopyable.hpp>

//点が領域の内にあるか外にあるかを調べるクラス
//AddPointした数が２点以下の時，IsPointInnerの戻り値は必ずtrue

class CPolygonalRegion
{
public:
  CPolygonalRegion();

  //四角形
  CPolygonalRegion(double dX1, double dY1, double dX2, double dY2);
  virtual ~CPolygonalRegion(void);

  CPolygonalRegion(const CPolygonalRegion &r);
  CPolygonalRegion & operator = (const CPolygonalRegion &r);

  size_t GetSize() const;
  bool IsPointInner(double x, double y) const;

  void AddPoint(double x, double y);
  void RemoveLast();
  void Reset();

  void GetPoints(std::vector<BoostVec> &rvVec) const;
  CvPoint GetPoint(size_t n) const;

  double GetXMin() const {return _dXMin;}
  double GetXMax() const {return _dXMax;}
  double GetYMin() const {return _dYMin;}
  double GetYMax() const {return _dYMax;}

  //座標系に平行な矩形領域を選択
  void SetRectangleRegion(double dX1, double dY1, double dX2, double dY2);

protected:

  double _dXMin;
  double _dXMax;
  double _dYMin;
  double _dYMax;


  CvMemStorage *_storage1;
  CvSeq* _pPointsOuter;

};

