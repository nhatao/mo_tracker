#include "StdAfx_MOTracking.h"
#include "MatrixFuncs.h"
#include "GridViewer.h"
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include "OutputDebugStream.h"
#include "Angle.h"
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable:4244)
#endif
#include <fltk/draw.h>
#include <fltk/compat/FL/fl_draw.H>
#ifdef _MSC_VER
#pragma warning (pop)
#endif

double SUnit::dLikelihoodThresh = 0.9;

using namespace std;

void CSlamMapViewerImpl::Resize(int nXGridNum, int nYGridNum) {

  _nXGridNum = nXGridNum;
  _nYGridNum = nYGridNum;

  int nX = _nXGridNum*(1+_nCellSize);
  int nY = _nYGridNum*(1+_nCellSize);
  resize(nY, nX);
}

void CSlamMapViewerImpl::Clear() {

//    fltk::setcolor(fltk::BLACK);
//    fltk::fillrect(0,0,w(),h());
  fl_rectf(0,0,w(),h(),FL_BLACK);
}

void CSlamMapViewerImpl::SetPoint(int x, int y, Fl_Color color, bool bLarge) {

  if (bLarge) {
        fl_rectf((_nYGridNum-2-y)*(_nCellSize+1), (_nXGridNum-2-x)*(_nCellSize+1), (_nCellSize+1)*3-1, (_nCellSize+1)*3-1, color);
    }
  else {
    fl_rectf((_nYGridNum-1-y)*(_nCellSize+1), (_nXGridNum-1-x)*(_nCellSize+1), _nCellSize, _nCellSize, color);
  }
}

void CSlamMapViewerImpl::SetPoint(int x, int y, uchar r, uchar g, uchar b, bool bLarge) {

  if (bLarge) {
    fl_rectf((_nYGridNum-2-y)*(_nCellSize+1), (_nXGridNum-2-x)*(_nCellSize+1), (_nCellSize+1)*3-1, (_nCellSize+1)*3-1, r, g, b);
  }
  else {
    fl_rectf((_nYGridNum-1-y)*(_nCellSize+1), (_nXGridNum-1-x)*(_nCellSize+1), _nCellSize, _nCellSize, r, g, b);
  }
}


int CSlamMapViewerImpl::GetScreenX(double y) const{
  return (int)round((_nYGridNum-y)*(_nCellSize+1));
}
int CSlamMapViewerImpl::GetScreenY(double x) const {
  return (int)round((_nXGridNum-x)*(_nCellSize+1));
}

double CSlamMapViewerImpl::GetMapX(int y) const {
  return _nYGridNum - y/(_nCellSize+1);
}
double CSlamMapViewerImpl::GetMapY(int x) const {
  return _nXGridNum - x/(_nCellSize+1);
}

CGridViewer2::CGridViewer2(boost::shared_ptr<const CGridSpace> pGridSpace, const char* sWindowName, int nCellSize) : CSlamMapViewerImpl(sWindowName, nCellSize) {

  _pGrid = pGridSpace;
  Resize(_pGrid->GetXGridNum(), _pGrid->GetYGridNum());

}

void CGridViewer2::DrawMap() {

  const SUnit* pGridUnit = _pGrid->GetObjects();
  int cnt1 = 0;
  int cnt2 = 0;

  for (int i=0; i<_nXGridNum; ++i) {
    for (int j=0; j<_nYGridNum; ++j) {
      const SUnit* pCurGrid = pGridUnit+_pGrid->GetArrayPos(i,j);
      double d = pCurGrid->dLikelihood;
      if (d < _dFreeThr) {
        SetPoint(i, j, 255, 0, 0);
      }
      else if (d > _dOccThr) {
        SetPoint(i, j, 255, 255, 255);
      }
      else {
        SetPoint(i, j, 0, 0, 0);
      }
    }
  }
}

void CGridViewer2::draw() {

  fl_font(FL_HELVETICA, 16);
  Clear();
  DrawMap();
  /*
  if (!_vMessages.empty()) {
    fl_color(255,64,64);
    for (size_t i=0; i<_vMessages.size(); ++i) {
      fl_draw(_vMessages[i].get<0>().c_str(), _vMessages[i].get<1>(), _vMessages[i].get<2>());
    }
    _vMessages.clear();
  }
  */
}
