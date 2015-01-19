#pragma once

#include <fltk/DoubleBufferWindow.h>
#include <fltk/compat/FL/Fl.H>
#include <vector>
#include "GridSpace.h"
#include <boost/tuple/tuple.hpp>

class CSlamMapViewerImpl : public fltk::DoubleBufferWindow {

public:

  CSlamMapViewerImpl(const char* sWindowName, int nCellSize = 2) : fltk::DoubleBufferWindow(500,200,sWindowName) {
    _nCellSize = nCellSize;
  };
  ~CSlamMapViewerImpl() {
  }
  void Clear();

protected:

  void SetPoint(int x, int y, Fl_Color color, bool bLarge=false);
  void SetPoint(int x, int y, uchar r, uchar g, uchar b, bool bLarge=false);
  //Map座標からスクリーン座標へ変換 xyが逆になるので注意
  int GetScreenX(double y) const;
  int GetScreenY(double x) const;
  //スクリーン座標からMap座標へ変換
  double GetMapX(int y) const;
  double GetMapY(int x) const;

  void Resize(int nX, int nY);

  int _nXGridNum;
  int _nYGridNum;
  int _nCellSize;
};

class CGridViewer2 : public CSlamMapViewerImpl
{

public:
  CGridViewer2(boost::shared_ptr<const CGridSpace> pGridSpace, const char* sWindowName, int nCellSize = 2);
  virtual ~CGridViewer2(void) {};

  virtual void DrawMap();

  void SetThr(double dOccThr, double dFreeThr) {
    _dOccThr = dOccThr;
    _dFreeThr = dFreeThr;
  }

protected:

  void draw();

  boost::shared_ptr<const CGridSpace> _pGrid;

  double _dOccThr;
  double _dFreeThr;

};
