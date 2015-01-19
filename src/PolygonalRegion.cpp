#include "StdAfx_MOTracking.h"
#include "PolygonalRegion.h"
#include <algorithm>


CPolygonalRegion::CPolygonalRegion(void)
{
  _storage1 = cvCreateMemStorage (0);
  _pPointsOuter = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), _storage1);

  _dXMin = DBL_MAX;
  _dXMax = -DBL_MAX;
  _dYMin = DBL_MAX;
  _dYMax = -DBL_MAX;
}

CPolygonalRegion::CPolygonalRegion(double x1, double y1, double x2, double y2) {

  _storage1 = cvCreateMemStorage (0);
  _pPointsOuter = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), _storage1);

  SetRectangleRegion(x1, y1, x2, y2);
}

void CPolygonalRegion::SetRectangleRegion(double x1, double y1, double x2, double y2) {

  cvClearSeq(_pPointsOuter);

  AddPoint(x1, y1);
  AddPoint(x2, y1);
  AddPoint(x2, y2);
  AddPoint(x1, y2);
}

CPolygonalRegion::~CPolygonalRegion(void)
{
  cvReleaseMemStorage(&_storage1);
}

void CPolygonalRegion::Reset() {

  cvClearMemStorage(_storage1);
  _pPointsOuter = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), _storage1);
}

CPolygonalRegion::CPolygonalRegion(const CPolygonalRegion &r) {

  _storage1 = cvCreateMemStorage (0);
  _pPointsOuter = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), _storage1);
  _dXMin = r._dXMin;
  _dXMax = r._dXMax;
  _dYMin = r._dYMin;
  _dYMax = r._dYMax;

  for (int i=0; i<r._pPointsOuter->total; ++i) {
    CvPoint *pt = (CvPoint*)cvGetSeqElem(r._pPointsOuter, i);
    cvSeqPush(_pPointsOuter, pt);
  }
}

CPolygonalRegion & CPolygonalRegion::operator = (const CPolygonalRegion &r) {

  Reset();
  _dXMin = r._dXMin;
  _dXMax = r._dXMax;
  _dYMin = r._dYMin;
  _dYMax = r._dYMax;

  for (int i=0; i<r._pPointsOuter->total; ++i) {
    CvPoint *pt = (CvPoint*)cvGetSeqElem(r._pPointsOuter, i);
    cvSeqPush(_pPointsOuter, pt);
  }
  return *this;
}


size_t CPolygonalRegion::GetSize() const {
  return _pPointsOuter->total;
}
bool CPolygonalRegion::IsPointInner(double x, double y) const {

  if (GetSize() <= 2) return false;
  else return (cvPointPolygonTest(_pPointsOuter, cvPoint2D32f(x,y), 0) >= 0);
}



void CPolygonalRegion::AddPoint(double x, double y) {

  auto Pos = cvPoint((int)x,(int)y);
  cvSeqPush(_pPointsOuter, &Pos);
//  cvSeqPush(_pPointsOuter, &cvPoint2D32f(x,y)); //なおすならcvGetSeqElemも．

  _dXMin = std::min(x, _dXMin);
  _dXMax = std::max(x, _dXMax);
  _dYMin = std::min(y, _dYMin);
  _dYMax = std::max(y, _dYMax);

}
void CPolygonalRegion::RemoveLast() {

  if (GetSize() != 0) {
    cvSeqPop(_pPointsOuter);

    //非効率?
    _dXMin = DBL_MAX;
    _dXMax = -DBL_MAX;
    _dYMin = DBL_MAX;
    _dYMax = -DBL_MAX;
    for (int i=0; i<_pPointsOuter->total; ++i) {
      CvPoint *pt = (CvPoint*)cvGetSeqElem(_pPointsOuter, i);
      double x = pt->x;
      double y = pt->y;
      _dXMin = std::min(x, _dXMin);
      _dXMax = std::max(x, _dXMax);
      _dYMin = std::min(y, _dYMin);
      _dYMax = std::max(y, _dYMax);
    }

  }
}

void CPolygonalRegion::GetPoints(std::vector<BoostVec> &rvVec) const {

  for (int i=0; i<_pPointsOuter->total; ++i) {
    BoostVec v(2);
    CvPoint *pt = (CvPoint*)cvGetSeqElem(_pPointsOuter, i);
    v(0) = pt->x; v(1) = pt->y;
    rvVec.push_back(v);
  }

}

CvPoint CPolygonalRegion::GetPoint(size_t n) const {

  if (n < GetSize()) {
    CvPoint *pt = (CvPoint*)cvGetSeqElem(_pPointsOuter, n);
    return *pt;
  }
  else {
    throw std::logic_error("CPolygonalRegion Out of range!");
  }

}
