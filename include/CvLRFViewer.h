#pragma once
#include <string>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include "MatrixFuncs.h"
#include "LaserData.h"

class CCvLRFViewer : boost::noncopyable
{
public:
  //dWidthLength: nWidth方向の表示距離
  CCvLRFViewer(const std::string& rsWindowname, int nWidth, int nHeight, double dWidthLength, double dXCenter=0, double dYCenter=0, double dWidthLengthMin=500, bool bAlpha = false);
  virtual ~CCvLRFViewer(void);
  void DrawPoint(double x, double y, const CvScalar& col, int nSize=1);
  void DrawCircle(double x, double y, const CvScalar& col, double dSize=1,int nThickness=1);
  void DrawString(const std::string& rStr, double x, double y, const CvScalar& col, bool bLRFCoord=true);
  void DrawLString(const std::string& rStr, double x, double y, const CvScalar& col, bool bLRFCoord=true);
  void DrawFreeSpace(const boost::shared_ptr<const CLaserData> &pLaserData, const CvScalar& col); //本来の座標に描画
  void DrawFreeSpace(const boost::shared_ptr<const CLaserData> &pLaserData, const CCoordinates3D &rCoords, const CvScalar& col); //本来の座標ではなく指定した座標に描画

  std::pair<int, int> GetScreenPos(double dX, double dY) const;
  std::pair<int, int> GetScreenPosWithCheck(double dX, double dY) const; //範囲外なら戻り値が-1
  std::pair<double, double> GetGlobalPos(double dScrX, double dScrY) const; //本来引数はint．サブピクセル対応のためdoubleを渡す

  void DrawCross(double x, double y, const CvScalar &col, int nSize=3, int nWidth=1);
  void DrawLine(double x1, double y1, double x2, double y2, const CvScalar& col, int nWidth=1);
  void DrawLine(double dist, double deg, const CvScalar& col, int nWidth=1);
  void DrawSegments2(std::vector<BoostVec>::const_iterator itBegin, std::vector<BoostVec>::const_iterator itEnd, const CvScalar& col, int nWidth=1);

  //MajorLen/MinorLenは半径
  void DrawEllipse(double dXCenter, double dYCenter, double dMajorLen, double dMinorLen, CAngle dOrigination, const CvScalar &col, int nThickness=1);

  void ResetViewer(const CvScalar& col = CV_RGB(0,0,0));
  virtual void ShowViewer() const;

  const IplImage* GetBufImage() const {return _BufImage;}
  IplImage* GetBufImage() {return _BufImage;}

  void SetDrawPosition(double dWidthLength, double dXCenter, double dYCenter);
  void SetWidthLength(double dWidthLength) {SetDrawPosition(dWidthLength, _dXCenter, _dYCenter);}
  void SetCenterPos(double dXCenter, double dYCenter) {SetDrawPosition(_dWidthLength, dXCenter, dYCenter);}

  int GetWidth()  const {return _nWidth;}
  int GetHeight() const {return _nHeight;}
  double GetScale() const {return _dScale;} //scale*pixel = length(mm)
  double GetWidthLength() const {return _dWidthLength;}
  double GetXCenter() const {return _dXCenter;}
  double GetYCenter() const {return _dYCenter;}

  template <class T>
  void DrawPoints2(const T& vec, const CvScalar& col, int nSize=1) {
    for (typename T::const_iterator it=vec.begin(); it!=vec.end(); ++it) {
      DrawPoint( (*it)[0], (*it)[1], col, nSize);
    }
  }
  template <class T>
  void DrawPoints(const T& itBegin, const T& itEnd, const CvScalar& col, int nSize=1) {
    for (T it = itBegin; it!=itEnd; ++it) {
      DrawPoint( (*it)[0], (*it)[1], col, nSize);
    }
  }
  template <class T>
  void DrawCrosses(const T& itBegin, const T& itEnd, const CvScalar& col, int nSize=3, int nWidth=1) {
    for (T it = itBegin; it!=itEnd; ++it) {
      DrawCross( (*it)[0], (*it)[1], col, nSize, nWidth);
    }
  }
  void DrawSquare(const CCoordinates2D& rCoCenter, double dLength, double dWidth, const CvScalar &rColor, int nWidth=1);
  void DrawSquare(double dCenterX, double dCenterY, double dRad, double dLength, double dWidth, const CvScalar &rColor, int nWidth=1);
  void DrawSquare(const BoostVec& rV1, const BoostVec& rV2, const CvScalar &rColor, int nWidth=1);


  void SaveImage(const std::string& filename) { cvSaveImage(filename.c_str(),_BufImage); }
  const std::string& GetWindowName() const {return _sWindowName;}

  bool GetPixelColor(double x, double y, CvScalar &rColor) const ; //指定した座標のピクセルの色を取得 範囲外ならfalseを返す

  double GetMinX() const {return (_nCX - _nHeight)*_dScale + _dXCenter;}
  double GetMaxX() const {return (_nCX - 0)*_dScale + _dXCenter;}
  double GetMinY() const {return (_nCY - _nWidth)*_dScale + _dYCenter;}
  double GetMaxY() const {return (_nCY - 0)*_dScale + _dYCenter;}
  void DrawGrid(double dGridWidth, const CvScalar &col); //等間隔に升目を描く

  //派生させて使うことも可能
  virtual void MouseFunc(int nEvent, int x, int y, int flags);
  virtual void TrackBarFunc(int nPos);

  void EnableCallback(); //マウスでのドラッグを可能にする
  void AddRangeTrackBar(double dMinRange = 500, double dMaxRange = 30*1000); //拡縮用のトラックバーをつける

  void SetMinWidthLength(double d) {_dMinWidthLength=d;}
  double GetMinWidthLength() const {return _dMinWidthLength;}

  //扇型
  void DrawFanShape(double dCenterX, double dCenterY,const CAngle& dAngle1, const CAngle& dAngle2, double dRadius, const CvScalar& col);

  void LockDrawMutex() const {
    _DrawMutex.lock();
  }
  void UnLockDrawMutex() const {
    _DrawMutex.unlock();
  }

protected:
  int _nCX;
  int _nCY;
  int _nWidth;
  int _nHeight;
  double _dXCenter;
  double _dYCenter;
  double _dWidthLength;

  double _dScale;
  std::string _sWindowName;
  IplImage* _BufImage;
  CvFont _Font;
  CvFont _LFont;

  std::vector<BoostVec> _vTemp;
  CCoordinates2D _TempCo;

  bool _bWindow;

  void SetCallBack();

  bool _bMouseFlag;
  int _nLastX;
  int _nLastY;

  bool _bHasTrackBar;
  int _nSliderVal;
  int _nMinRange;

  double _dMinWidthLength;

  mutable boost::mutex _DrawMutex;

};
