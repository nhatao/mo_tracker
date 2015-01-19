#include "StdAfx_MOTracking.h"

#include "CvLRFViewer.h"
#include <iostream>
#include <boost/tuple/tuple.hpp>

using namespace std;

CCvLRFViewer::CCvLRFViewer(const std::string& rsWindowname, int nWidth, int nHeight, double dWidthLength, double dXCenter, double dYCenter,double dWidthLengthMin,  bool bAlpha)
{
  int nCH = 3;
  if (bAlpha) nCH=4;
  _BufImage = cvCreateImage(cvSize(nWidth,nHeight), IPL_DEPTH_8U, nCH);

  _sWindowName = rsWindowname;
  _nWidth = nWidth;
  _nHeight = nHeight;
  _nCY = nWidth/2;
  _nCX = nHeight/2;
  _dMinWidthLength = dWidthLengthMin;
  SetDrawPosition(dWidthLength, dXCenter, dYCenter);

  cvInitFont(&_Font, CV_FONT_HERSHEY_PLAIN,1.2,1.2,0,2);
  cvInitFont(&_LFont, CV_FONT_HERSHEY_PLAIN,2.4,2.4,0,2);
  ResetViewer();

  _vTemp.resize(4);
  for (auto itv=_vTemp.begin(); itv!=_vTemp.end(); ++itv) itv->resize(2);

  _bWindow = false;
  _bHasTrackBar = false;
}

CCvLRFViewer::~CCvLRFViewer(void)
{
  cvReleaseImage(&_BufImage);
  if (_bWindow) 
    cvDestroyWindow(_sWindowName.c_str());
}


void CCvLRFViewer::SetDrawPosition(double dWidthLength, double dXCenter, double dYCenter) {
  if (dWidthLength < _dMinWidthLength) {
    cout << __FUNCTION__ << " dWidthLength Too Small: " << dWidthLength << endl;
    dWidthLength = _dMinWidthLength;
  }

  _dXCenter = dXCenter;
  _dYCenter = dYCenter;
  _dScale = dWidthLength/_nWidth;
  _dWidthLength = dWidthLength;
}

std::pair<int, int> CCvLRFViewer::GetScreenPosWithCheck(double dX, double dY) const {

  int ny = (int)round(_nCX - (dX-_dXCenter)/_dScale);
  int nx = (int)round(_nCY - (dY-_dYCenter)/_dScale);
  if ((nx >= _nWidth)  || (nx < 0)) nx = -1;
  if ((ny >= _nHeight) || (ny < 0)) ny = -1;
  return make_pair(nx,ny);
}


std::pair<int, int> CCvLRFViewer::GetScreenPos(double dX, double dY) const {

  int ny = (int)round(_nCX - (dX-_dXCenter)/_dScale);
  int nx = (int)round(_nCY - (dY-_dYCenter)/_dScale);
  return make_pair(nx,ny);
}

std::pair<double, double> CCvLRFViewer::GetGlobalPos(double dScrX, double dScrY) const {

  double dX = (_nCX - dScrY)*_dScale + _dXCenter;
  double dY = (_nCY - dScrX)*_dScale + _dYCenter;
  return make_pair(dX,dY);
}



void CCvLRFViewer::DrawPoint(double x, double y, const CvScalar& col, int nSize) {

  int nx, ny;
  boost::tie(nx, ny) = GetScreenPos(x,y);

  cvRectangle(_BufImage, cvPoint(nx-nSize/2, ny-nSize/2), cvPoint(nx+nSize/2,ny+nSize/2), col);
  /*
  if (nSize>1) {
    cvRectangle(_BufImage, cvPoint(nx-nSize/2, ny-nSize/2), cvPoint(nx+nSize/2,ny+nSize/2), col);
  else {
    cvRectangle(_BufImage, cvPoint(nx-nSize/2, ny-nSize/2), cvPoint(nx+nSize/2,ny+nSize/2), col);
  }
  */
}

void CCvLRFViewer::DrawCircle(double x, double y, const CvScalar& col, double dSize, int nThickness) {
  if (dSize <= 0) return;
  int nSize = (int)round(dSize/_dScale);
  if (nSize <= 1) DrawPoint(x,y,col);
  int nx, ny;
  boost::tie(nx, ny) = GetScreenPos(x,y);


//void circle( BoostMat& img, Point center, int radius,const Scalar& color, int thickness, int line_type, int shift )
//    CV_Assert( radius >= 0 && thickness <= 255 && 0 <= shift && shift <= XY_SHIFT );

  if (nSize >= 0 && nThickness <= 255) {
    cvCircle(_BufImage, cvPoint(nx, ny), nSize, col, nThickness);
  }
  else {
    cout << __FUNCTION__ << " Something Wrong. Size=" << nSize << " Thickness=" << nThickness << endl;
    cout << "Argument: x=" << x << " y=" << y << " dSize=" << dSize << " nThickness=" << nThickness << endl;
  }

}

void CCvLRFViewer::DrawFanShape(double dCenterX, double dCenterY, const CAngle& dAngle1, const CAngle& dAngle2, double dRadius, const CvScalar& col) {

  CvPoint pts[130];
  boost::tie(pts[0].x,pts[0].y) = GetScreenPos(dCenterX, dCenterY);

  CAngle dCurAngle = dAngle1;
  size_t nPos = 1;
  while (dCurAngle < dAngle2) {
    double dX1 = dCenterX+dRadius*cos(dCurAngle);
    double dY1 = dCenterY+dRadius*sin(dCurAngle);
    boost::tie(pts[nPos].x,pts[nPos].y) = GetScreenPos(dX1, dY1);
    dCurAngle += Deg2Rad(3);
    ++nPos;
  }
  double dX1 = dCenterX+dRadius*cos(dAngle2);
  double dY1 = dCenterY+dRadius*sin(dAngle2);
  boost::tie(pts[nPos].x,pts[nPos].y) = GetScreenPos(dX1, dY1);

  cvFillConvexPoly (_BufImage, pts, nPos+1, col);
}

void CCvLRFViewer::DrawFreeSpace(const boost::shared_ptr<const CLaserData> &pLaserData, const CvScalar& col) {

  DrawFreeSpace(pLaserData, *pLaserData->GetCo(), col);
}

void CCvLRFViewer::DrawFreeSpace(const boost::shared_ptr<const CLaserData> &pLaserData, const CCoordinates3D &rCoords, const CvScalar& col) {

  CAngle dAngle = pLaserData->GetProperty()._dFirstAngle - (pLaserData->GetProperty()._dReso/2) + rCoords.GetYaw();
  double dCX = rCoords.GetX();
  double dCY = rCoords.GetY();
  for (auto pDist = pLaserData->GetRawData().begin(); pDist != pLaserData->GetRawData().end(); ++pDist, dAngle+=pLaserData->GetProperty()._dReso) {

    double d = *pDist;
    if (d < pLaserData->GetProperty()._dMinRange) { //無効点
      d = pLaserData->GetProperty()._dMaxRange;
    }
    else if (d > pLaserData->GetProperty()._dMaxRange) {
      d = pLaserData->GetProperty()._dMaxRange;
    }   
    DrawFanShape(dCX, dCY, dAngle, dAngle+pLaserData->GetProperty()._dReso, d, col);
  }
}




void CCvLRFViewer::DrawLine(double x1, double y1, double x2, double y2, const CvScalar& col, int nWidth) {

  int nx1, ny1, nx2, ny2;
  boost::tie(nx1, ny1) = GetScreenPos(x1,y1);
  boost::tie(nx2, ny2) = GetScreenPos(x2,y2);
  cvLine(_BufImage, cvPoint(nx1, ny1), cvPoint(nx2, ny2), col, nWidth);
}

void CCvLRFViewer::DrawLine(double dist, double deg, const CvScalar& col, int nWidth) {

  double xa = dist*cos(deg);
  double ya = dist*sin(deg);
  double x1 = xa - _dScale*_nHeight*sin(deg);
  double y1 = ya + _dScale*_nHeight*cos(deg);
  double x2 = xa + _dScale*_nHeight*sin(deg);
  double y2 = ya - _dScale*_nHeight*cos(deg);
  DrawLine(x1, y1, x2, y2, col, nWidth);  
}

void CCvLRFViewer::DrawCross(double x, double y, const CvScalar& col, int nSize, int nWidth) {

  int nx, ny;
  boost::tie(nx, ny) = GetScreenPos(x,y);

  if ((nSize % 2) != 0) --nSize;
  nSize/=2;
  cvLine(_BufImage, cvPoint(nx-nSize, ny), cvPoint(nx+nSize, ny), col, nWidth);
  cvLine(_BufImage, cvPoint(nx, ny-nSize), cvPoint(nx, ny+nSize), col, nWidth);
}


void CCvLRFViewer::ResetViewer(const CvScalar& col) {
  cvRectangle(_BufImage, cvPoint(0,0), cvPoint(_nWidth,_nHeight), col, -1);
}

void CCvLRFViewer::ShowViewer() const{
  if (_bWindow) cvNamedWindow(_sWindowName.c_str(), 1);
  cvShowImage(_sWindowName.c_str(), _BufImage);
}

void CCvLRFViewer::DrawString(const std::string& rStr, double x, double y, const CvScalar& col, bool bLRFCoord) {
  if (bLRFCoord) {
    int nx, ny;
    boost::tie(nx, ny) = GetScreenPos(x,y);
    cvPutText(_BufImage, rStr.c_str(), cvPoint (nx,ny), &_Font, col);
  }
  else 
    cvPutText(_BufImage, rStr.c_str(), cvPoint (int(x),int(y)), &_Font, col);
}
void CCvLRFViewer::DrawLString(const std::string& rStr, double x, double y, const CvScalar& col, bool bLRFCoord) {
  if (bLRFCoord) {
    int nx, ny;
    boost::tie(nx, ny) = GetScreenPos(x,y);
    cvPutText(_BufImage, rStr.c_str(), cvPoint (nx,ny), &_LFont, col);
  }
  else 
    cvPutText(_BufImage, rStr.c_str(), cvPoint (int(x),int(y)), &_LFont, col);
}


void CCvLRFViewer::DrawSegments2(std::vector<BoostVec>::const_iterator  itBegin, std::vector<BoostVec>::const_iterator itEnd, const CvScalar& col, int nWidth) {

  if (itBegin == itEnd) return;
  std::vector<BoostVec>::const_iterator itLast = itEnd; --itLast;
  if (itBegin == itLast) return;
  for (std::vector<BoostVec>::const_iterator  it = itBegin; it != itLast; ++it) {
    std::vector<BoostVec>::const_iterator  it2 = it; ++it2;
    DrawLine((*it)[0], (*it)[1], (*it2)[0], (*it2)[1], col, nWidth);
  }
}

bool CCvLRFViewer::GetPixelColor(double x, double y, CvScalar &rColor) const {
  int nX, nY;
  boost::tie(nX, nY) = GetScreenPosWithCheck(x,y);
  if ((nX<0) || (nY<0)) return false;

  rColor = cvGet2D(_BufImage, nY, nX);
  return true;
}

void CCvLRFViewer::DrawSquare(double dCenterX, double dCenterY, double dRad, double dWidth, double dHeight, const CvScalar &rColor, int nWidth) {

  _TempCo.SetPos(dCenterX, dCenterY);
  _TempCo.SetRotation(dRad);
  DrawSquare(_TempCo, dWidth, dHeight, rColor, nWidth);
}

void CCvLRFViewer::DrawSquare(const CCoordinates2D& rCoCenter, double dWidth, double dHeight, const CvScalar &rColor, int nWidth) {
  
  _vTemp[0](0) =  dWidth/2;   _vTemp[0](1) =  dHeight/2;  
  _vTemp[1](0) = -dWidth/2;   _vTemp[1](1) =  dHeight/2;  
  _vTemp[2](0) = -dWidth/2;   _vTemp[2](1) = -dHeight/2;  
  _vTemp[3](0) =  dWidth/2;   _vTemp[3](1) = -dHeight/2;  

  for (auto itv = _vTemp.begin(); itv != _vTemp.end(); ++itv) {
    rCoCenter.TransformVector(*itv);
  }
  for (auto itv = _vTemp.begin(); itv != _vTemp.end(); ++itv) {
    auto itv2 = itv; ++itv2;
    if (itv2 == _vTemp.end()) itv2 = _vTemp.begin();

    DrawLine((*itv)(0), (*itv)(1), (*itv2)(0), (*itv2)(1), rColor, nWidth);
  }
}

void CCvLRFViewer::DrawSquare(const BoostVec& rV1, const BoostVec& rV2, const CvScalar &rColor, int nWidth) {

  double dX1 = min(rV1(0), rV2(0));
  double dX2 = max(rV1(0), rV2(0));
  double dY1 = min(rV1(1), rV2(1));
  double dY2 = max(rV1(1), rV2(1));
  DrawLine(dX1, dY1, dX1, dY2, rColor, nWidth);
  DrawLine(dX1, dY2, dX2, dY2, rColor, nWidth);
  DrawLine(dX2, dY2, dX2, dY1, rColor, nWidth);
  DrawLine(dX2, dY1, dX1, dY1, rColor, nWidth);
}


void CCvLRFViewer::DrawGrid(double dGridWidth, const CvScalar &col) {

  double dScreenGridWidth = dGridWidth/_dScale;

  double dx0 = GetMinX();
  double dy0 = GetMinY();
  double dx1 = GetMaxX();
  double dy1 = GetMaxY();
  dx0 = floor(dx0/dGridWidth) * dGridWidth;
  dy0 = floor(dy0/dGridWidth) * dGridWidth;


  for (double dx = dx0; dx < dx1; dx+=dGridWidth) {
    DrawLine(dx, dy0, dx, dy1, col);
  }
  for (double dy = dy0; dy < dy1; dy+=dGridWidth) {
    DrawLine(dx0, dy, dx1, dy, col);
  }
}


void CCvLRFViewerFuncHelper(int nEvent, int x, int y, int flags, void* param) {

  CCvLRFViewer *p = (CCvLRFViewer*)param;
  p->MouseFunc(nEvent, x, y, flags);
}

void CCvLRFViewer::SetCallBack() { 
  cvSetMouseCallback(GetWindowName().c_str(), &CCvLRFViewerFuncHelper, this);
}


void CCvLRFViewer::DrawEllipse(double dXCenter, double dYCenter, double dMajorLen, double dMinorLen, CAngle dOrigination, const CvScalar &col, int nThickness) {

  int ncX, ncY;
  boost::tie(ncX, ncY) = GetScreenPos(dXCenter, dYCenter);
  int nLen1 = (int)round(dMajorLen/_dScale);
  int nLen2 = (int)round(dMinorLen/_dScale);

  if (nLen1 >= 0 && nLen1 >= 0 && nThickness <= 255) {
    cvEllipse(GetBufImage(), cvPoint(ncX, ncY), cvSize(nLen2, nLen1), -(int)round(dOrigination.get_deg()), 0, 360, col, nThickness);
  }
  else {
    cout << "something wrong !" << endl;
    cout << "x,y=" << dXCenter <<"," <<  dYCenter << endl;
    cout << "nCX,nCY= " << ncX << "," << ncY << endl;
    cout << "rLen=" << dMajorLen << "," << dMinorLen << endl;
    cout << "nLen=" << nLen1 << "," << nLen2 << endl;
    cout << "thickness=" << nThickness << endl;
  }
}

void CCvLRFViewer::EnableCallback() {

  _bMouseFlag = false;
  _nLastX = 0;
  _nLastY = 0;
  ShowViewer();
  SetCallBack();
}

void CCvLRFViewer::MouseFunc(int nEvent, int x, int y, int flags) {

  if ( (x<0) || (y<0) || (x>=_nWidth) || (y>=_nHeight)) {
    _bMouseFlag=false;
    return;
  }

  switch(nEvent)
  {
    case CV_EVENT_LBUTTONDOWN:
    _bMouseFlag = true;
    _nLastX = x;
    _nLastY = y;
    break;
    case CV_EVENT_LBUTTONUP:
    _bMouseFlag=false;
    break;
  }

  if (nEvent == CV_EVENT_MOUSEMOVE && _bMouseFlag) {

    double dScale = GetScale();
    double dXCenter = GetXCenter();
    double dYCenter = GetYCenter();
    dYCenter -= (_nLastX-x)*dScale;
    dXCenter -= (_nLastY-y)*dScale;
    SetCenterPos(dXCenter, dYCenter);
    _nLastX = x;
    _nLastY = y;
  }

}

void TrackBarCallBackHelper(int nPos, void *param) {
  CCvLRFViewer *p = (CCvLRFViewer*)param;
  p->TrackBarFunc(nPos);
}

void CCvLRFViewer::TrackBarFunc(int nPos) {

  int nPos2 = max(nPos, _nMinRange);
  SetWidthLength(nPos2);
}

void CCvLRFViewer::AddRangeTrackBar(double dMinRange, double dMaxRange) {

  _dMinWidthLength = min(dMinRange, _dMinWidthLength);
  _nMinRange = (int)dMinRange;
  _nSliderVal = (int)_dWidthLength;
  cvCreateTrackbar2("Range", _sWindowName.c_str(), &_nSliderVal, (int)dMaxRange, 
    &TrackBarCallBackHelper, this);
}

