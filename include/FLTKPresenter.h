#pragma once
#include "MOTrackerFramework.h"
#include "PolygonalRegion.h"
#include "CvLRFViewer.h"
#include "SJPDATrackerConfig.h"
#include <queue>
#include <boost/circular_buffer.hpp>
#include <boost/tuple/tuple.hpp>
#include <fltk/DoubleBufferWindow.h>
#include "TrackInitialization.h"

class CTrackerResultView : public CMOTrackerView {

public:

  CTrackerResultView(boost::shared_ptr<CMOTrackerViewModel> pViewModel)
    : CMOTrackerView(pViewModel)
  {
  }
  virtual ~CTrackerResultView() {
  }
 
  virtual void ClearWindow(uint8_t r, uint8_t g, uint8_t b) = 0;
  virtual void DrawCylinder(const BoostVec &rPos, double dRadius,  double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold) = 0;
  virtual void DrawEllipticCylinder(const BoostVec &rPos, double dR1, double dR2, CAngle dAngle,  double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold) = 0;
  virtual void DrawCuboid(const BoostVec &rPos, double dLength, double dWidth, CAngle dAngle,  double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold) = 0;

  virtual void DrawSpeedArow(const BoostVec &rPos, const BoostVec &rVec, uint8_t r, uint8_t g, uint8_t b) = 0;
  virtual void DrawFreeSpace(const boost::shared_ptr<const CLaserData> &pLaserData, uint8_t r, uint8_t g, uint8_t b) = 0;
  virtual void DrawPoints(const boost::shared_ptr<const CLaserData> &pLaserData, uint8_t r, uint8_t g, uint8_t b, int nSize) = 0;
  virtual void DrawLine(const BoostVec &v1, const BoostVec &v2, uint8_t r, uint8_t g, uint8_t b, int nWidth) = 0;
  virtual void DrawPoint(const BoostVec &vPos, uint8_t r, uint8_t g, uint8_t b, int nSize, bool bBold) = 0;
  virtual void DrawSensor(const boost::shared_ptr<const CCascadedCoords> &pLRFCo, const std::string &rSensor) = 0;
  virtual void DrawMessage(const std::string &rsMsg, uint8_t r, uint8_t g, uint8_t b) = 0;
  virtual void DrawHistory(const std::vector<boost::shared_ptr<const SMovingObjectStatus> > & rHistory, size_t nDrawNum, uint8_t r, uint8_t g, uint8_t b) = 0; //obsolete
  virtual void DrawHistory(const boost::circular_buffer<boost::shared_ptr<const SMovingObjectStatus> > & rHistory, size_t nDrawNum, uint8_t r, uint8_t g, uint8_t b) = 0; //obsolete
  virtual void DrawCaption(const BoostVec &vPos, const std::string &rsMsg, uint8_t r, uint8_t g, uint8_t b) = 0;

  virtual void ShowWindow() = 0;
  virtual void DrawRegion(const CPolygonalRegion &rRegion, double dIntervalPixel, uint8_t r, uint8_t g, uint8_t b) = 0;

  virtual void SetViewerCenter(double x, double y)=0; //この点を中央に持ってくる
  virtual void SaveImage(const std::string &rsFileName)=0;

protected:
};


class CTrackerResultViewModel : public CMOTrackerViewModel {
public:

  CTrackerResultViewModel(CMOTrackerPresenter* pPresenter): CMOTrackerViewModel(pPresenter) {
    _bDragging = false;
    _nRollbackFrame = 0;
    _nLastFrame = -1;
    _bChaseLRF = false;
    _nHistoryDrawLen = 50;
    _nParticleDrawMode = 0;
    _nDebugPFNum = 0;
    _nDebugParticleNum = 0;
    _vTemp.resize(2);
    _vTemp2.resize(2);

  }
  virtual ~CTrackerResultViewModel() {}
  virtual void SetCurrentResult(const CTrackerHistory &rResult) {
    _History = rResult;
  }
  void DrawTrackerResult(CTrackerResultView* pView);

  //GUIのキー・マウス入力
  virtual void KeyPressed(CTrackerResultView* pView, int nKeyCode);
  void MousePressed(CTrackerResultView* pView, bool bButton1, bool bButton2, bool bButton3, int nX, int nY);
  void MouseMoved(CTrackerResultView* pView, bool bButton1, bool bButton2, bool bButton3, int nX, int nY);
  void MouseReleased(CTrackerResultView* pView, bool bButton1, bool bButton2, bool bButton3, int nX, int nY);

  void SetChaseLRF(bool b) {
    _bChaseLRF = b;
  }
  bool GetChaseLRF() const {return _bChaseLRF;}
  void SetHistoryDrawLen(int n) {
    _nHistoryDrawLen = n;
  }
  int GetHistoryDrawLen() const {
    return _nHistoryDrawLen;
  }

  void ToggleParticleDrawMode() {
    ++_nParticleDrawMode;
    if (_nParticleDrawMode == 3) _nParticleDrawMode = 0;
    _nDebugPFNum = 0;
    _nDebugParticleNum = 0;
  }

protected:

  CTrackerHistory _History;
  SLRFProperty _Prop;

  bool _bDragging;
  double _dDragStartX;
  double _dDragStartY;
  double _dDragLastX;
  double _dDragLastY;

  int _nRollbackFrame; //0のとき最新の結果．値が増えるほど前の結果
  int _nLastFrame;
  bool _bChaseLRF;
  int _nHistoryDrawLen;


  int _nParticleDrawMode; //0: Filteredを描画 1: Predictedを描画 2: DebugMode(Predictedを詳細情報込みで１つずつ描画, "@",":"でPF変更, "[","]"でパーティクル変更)
  int _nDebugPFNum;
  int _nDebugParticleNum;
  BoostVec _vTemp;
  BoostVec _vTemp2;

  virtual void DrawBeforeObjects(CTrackerResultView* pView, const boost::shared_ptr<const STrackerResult> &pTrackerResult,const std::vector<boost::shared_ptr<const CLaserData> > &rvpLaserData);
  virtual void DrawAfterObjects(CTrackerResultView* pView, const boost::shared_ptr<const STrackerResult> &pTrackerResult,const std::vector<boost::shared_ptr<const CLaserData> > &rvpLaserData);

  void DrawParticle(CTrackerResultView* pView, const BoostVec &rvParticle, int nType, uint8_t r, uint8_t g, uint8_t b, bool bBold, bool bDrawSpeed=false);
  int TypeNameToInt(const std::string &rsType);
};

#include <fltk/Window.h>
#include <fltk/Button.h>
#include <fltk/draw.h>
#include <fltk/ValueSlider.h>

class CFLTKWindowView : public CTrackerResultView , public fltk::Window  
{

public:

  CFLTKWindowView(boost::shared_ptr<CMOTrackerViewModel> pViewModel, boost::shared_ptr<CCvLRFViewer> pViewer);
  virtual ~CFLTKWindowView();

  virtual void Update();
  virtual void ShowWindow();

  virtual void ClearWindow(uint8_t r, uint8_t g, uint8_t b);
  virtual void DrawCylinder(const BoostVec &rPos, double dRadius,  double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold);
  virtual void DrawEllipticCylinder(const BoostVec &rPos, double dR1, double dR2, CAngle dAngle,  double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold);
  virtual void DrawCuboid(const BoostVec &rPos, double dLength, double dWidth, CAngle dAngle,  double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold);

  virtual void DrawSpeedArow(const BoostVec &rPos, const BoostVec &rVec, uint8_t r, uint8_t g, uint8_t b);
  virtual void DrawFreeSpace(const boost::shared_ptr<const CLaserData> &pLaserData, uint8_t r, uint8_t g, uint8_t b);
  virtual void DrawPoints(const boost::shared_ptr<const CLaserData> &pLaserData, uint8_t r, uint8_t g, uint8_t b, int nSize);
  virtual void DrawLine(const BoostVec &v1, const BoostVec &v2, uint8_t r, uint8_t g, uint8_t b, int nWidth);
  virtual void DrawPoint(const BoostVec &vPos, uint8_t r, uint8_t g, uint8_t b, int nSize, bool bBold);
  virtual void DrawSensor(const boost::shared_ptr<const CCascadedCoords> &pLRFCo, const std::string &rSensor);
  virtual void DrawHistory(const std::vector<boost::shared_ptr<const SMovingObjectStatus> > & rHistory, size_t nDrawNum, uint8_t r, uint8_t g, uint8_t b); //obsolete
  virtual void DrawHistory(const boost::circular_buffer<boost::shared_ptr<const SMovingObjectStatus> > & rHistory, size_t nDrawNum, uint8_t r, uint8_t g, uint8_t b);
  virtual void DrawMessage(const std::string &rsMsg, uint8_t r, uint8_t g, uint8_t b);
  virtual void DrawRegion(const CPolygonalRegion &rRegion, double dIntervalPixel, uint8_t r, uint8_t g, uint8_t b);
  virtual void DrawCaption(const BoostVec &vPos, const std::string &rsMsg, uint8_t r, uint8_t g, uint8_t b);
  virtual void SetViewerCenter(double x, double y);
  virtual void SaveImage(const std::string &rsFileName);


  int handle(int nEvent);

  double GetSliderVal() {return _pSlider->value();}
  struct SDragState {
    enum EState {
      eStart, eDrag, eEnd, eNothing
    };
    EState nState;
    double dStartX;
    double dStartY;
    double dLastX;
    double dLastY;
    double dCurrentX;
    double dCurrentY;
  };

  const SDragState& GetCurrentDragState() const {return _CurrentState;}
  void ResetState() {_CurrentState.nState = SDragState::eNothing;}
  void ResetViewerRange();

  std::pair<double, double> GetMouseXY() const {return std::make_pair(_dCurX, _dCurY);}

  boost::shared_ptr<const CCvLRFViewer> GetViewer() const {
    return _pLRFViewer;
  }
  boost::shared_ptr<CCvLRFViewer> GetViewer() {
    return _pLRFViewer;
  }

  virtual void SetMouseClickedPos(int x, int y) {
    boost::tie(_dCurX, _dCurY) = _pLRFViewer->GetGlobalPos(x,y);
  }

protected:
  boost::shared_ptr<CCvLRFViewer> _pLRFViewer;
  IplImage* _BufImage;
  int _nMsgCount;

  void draw();
  void Init();

  double _dOrgXCenter;
  double _dOrgYCenter;
  double _dOrgWidthLength;
  double _XStart;
  double _YStart;

  fltk::ValueSlider* _pSlider;
  fltk::Button* _pResetButton;

  void StartDrag();
  void MoveDrag();
  void EndDrag();
//  virtual void HandleKey();
  
  SDragState _CurrentState;
  double _dCurX;
  double _dCurY;
};



class CFLTKPresenter :
  public CMOTrackerPresenter
{
public:
  CFLTKPresenter(CMOTrackerFramework *pFramework);
  virtual ~CFLTKPresenter(void);

  virtual void DoLoopProc(const CTrackerHistory &rHistory);
  void Initialize(const SJPDATrackerConfig &Config);

  void FinishProgram();
  void SaveIniterBG();

  void MakeGridViewer(boost::shared_ptr<const CTrackInitializationWithProperty>  pIniter);
  virtual void ProcKeyCode(int nKey);

protected:

  boost::shared_ptr<CCvLRFViewer> _pMainViewer;
  SJPDATrackerConfig _FirstConfig;
  boost::shared_ptr<const CTrackInitializationWithProperty> GetInitializer();
  std::vector<boost::shared_ptr<fltk::DoubleBufferWindow> > _vpGridViewers;
};


class CTrackInitializationUsingPolarGrid;
class CPolarGridViewer : public fltk::DoubleBufferWindow
{

public:
  CPolarGridViewer(
    boost::shared_ptr<const CTrackInitializationUsingPolarGrid> pGrid, const std::string &sWindowName, int nCellSize = 1);
  virtual ~CPolarGridViewer(void);

  virtual void DrawMap();

protected:

  boost::shared_ptr<const CTrackInitializationUsingPolarGrid> _pGrid;
  void draw();

  size_t _nCellSize;
//  typedef boost::base_from_member<std::string> _sWindowName; //fltk::DoubleBufferWindowを呼び出す前にこれを初期化する必要があるので，boost::base_from_memberを使う
  std::string _sWindowName;

  std::vector<double> _aArrayCopied;
};
