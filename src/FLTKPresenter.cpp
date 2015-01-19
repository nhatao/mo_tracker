#include "StdAfx_MOTracking.h"
#include "FLTKPresenter.h"
#include "mmtimer.h"
#include <fltk/run.h>
#include <fltk/events.h>
#include <fltk/run.h>
#include <fltk/FL_API.h>
#include "MOTracker.h"
#include "JPDATracker.h"
#include "ColorCout.h"
#include <boost/filesystem.hpp>
#include "TrackInitializationUsingGrid.h"
#include "TrackInitializationUsingPolarGrid.h"
#include "GridViewer.h"
#include "FastMath.h"
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable:4244)
#endif
#include <fltk/compat/FL/Fl.H>
#include <fltk/compat/FL/fl_draw.H>
#ifdef _MSC_VER
#pragma warning (pop)
#endif

#include <boost/assign.hpp>
using namespace boost::assign;
using namespace std;

//xを押されても閉じないようにする
void window_callback_donothing(fltk::Widget* widget, void*) {
}

CFLTKPresenter::CFLTKPresenter(CMOTrackerFramework *pFramework) : CMOTrackerPresenter(pFramework) {
}
CFLTKPresenter::~CFLTKPresenter(void) {
}

void CFLTKPresenter::MakeGridViewer(boost::shared_ptr<const CTrackInitializationWithProperty>  _pInitializer) {

  if ((!_vpGridViewers.empty()) || (!_FirstConfig._bDrawGrid)) return;

  ccout->SetColor(ColorCout::eGreen);
  ccout << "DrawGrid: " << std::boolalpha << _FirstConfig._bDrawGrid << endl;

  if (_FirstConfig._bDrawGrid) {
    int _nID = 0;

    if (_FirstConfig._nIniterType == 0) {
      const CTrackInitializationUsingGrid* pGrid = 
        dynamic_cast<const CTrackInitializationUsingGrid*>(_pInitializer.get());
      if (pGrid) {
        ostringstream oss1; oss1 << "GridViewer #" << _nID;

        auto pGridViewer = boost::shared_ptr<CGridViewer2>(new CGridViewer2(pGrid->GetGrid(),"GridViewer",2));
        pGridViewer->position(650,40); 
        pGridViewer->show();
        pGridViewer->redraw();
        pGridViewer->SetThr(pGrid->GetObstacleThr(), pGrid->GetFreeThr());
        _vpGridViewers.push_back(pGridViewer);
      }
      else {
        cout << "unknown error in " << __FUNCTION__ << endl;
      }
    }
    else if (_FirstConfig._nIniterType == 1) {
      ostringstream oss1; oss1 << "GridViewer #" << _nID;
      auto pGridViewer = boost::shared_ptr<CPolarGridViewer>(new CPolarGridViewer(boost::dynamic_pointer_cast<const CTrackInitializationUsingPolarGrid>(_pInitializer),oss1.str(),2));
      pGridViewer->position(650,40); 
      pGridViewer->show();
      pGridViewer->redraw();
      _vpGridViewers.push_back(pGridViewer);

    }
    else {

    }
  }
}

void CFLTKPresenter::Initialize(const SJPDATrackerConfig &Config ) {

  //Track Viewer Init
  double dXLen = Config._dViewerRange;
  double dYLen = Config._dViewerRange;
  double dCX = 0;
  double dCY = 0;

  double dViewerY = Config._dViewerSizeY;
  double dViewerX = dViewerY * dYLen/dXLen;
  //8の倍数に揃える
  int nViewerX = 8*( (int)(round(dViewerX/8.0)) );
  int nViewerY = 8*( (int)(round(dViewerY/8.0)) );


  _pMainViewer = boost::shared_ptr<CCvLRFViewer> 
    (new CCvLRFViewer ("MOTracker", nViewerX, nViewerY, dYLen, dCX, dCY));

  auto p = new CTrackerResultViewModel(this);
  p->SetChaseLRF(Config._bChaseLRF);
  boost::shared_ptr<CMOTrackerViewModel> pViewModel(p);

  CFLTKWindowView *pView = new CFLTKWindowView(pViewModel,_pMainViewer);
  pView->callback(window_callback_donothing);
  
  _vpViewModels.push_back(pViewModel);
  _vpViews.push_back(boost::shared_ptr<CMOTrackerView>(pView));

  pView->show();

  _FirstConfig = Config;
}


boost::shared_ptr<const CTrackInitializationWithProperty> CFLTKPresenter::GetInitializer() {

  boost::shared_ptr<const CMOTracker> pTracker = _pModel->GetTracker();
  boost::shared_ptr<const CJPDATracker> pTracker3 = boost::dynamic_pointer_cast<const CJPDATracker>(pTracker);
  if (pTracker3) {
    return pTracker3->GetInitializer();
  }
  return boost::shared_ptr<const CTrackInitializationWithProperty> ();
}


void CFLTKPresenter::DoLoopProc(const CTrackerHistory &rHistory) {

  for (auto it=_vpViewModels.begin(); it!=_vpViewModels.end(); ++it) {
    (*it)->SetCurrentResult(rHistory);
  }
  for (auto it=_vpViews.begin(); it!=_vpViews.end(); ++it) {
    (*it)->Update();
  }
  //todo view化
  if (!_vpGridViewers.empty()) {
    for (auto it=_vpGridViewers.begin(); it!=_vpGridViewers.end(); ++it) {
      (*it)->redraw();
    }
  }
  else {
    //Initializerがすぐには作られないため
    auto pIniter = GetInitializer();
    if (pIniter) {
      MakeGridViewer(pIniter);
    }
  }

  fltk::check();
  int nKey = fltk::event_key();
  if (nKey == fltk::ReturnKey) {
    FinishProgram();
  }
}

void CFLTKPresenter::FinishProgram() {
  _pModel->FinishProgram();
}


void CFLTKPresenter::SaveIniterBG(){

  auto pIniter = GetInitializer();
  if (!pIniter) {
    return;
  }

  int n=0;
  while(true) {
    boost::filesystem::path dir("dat");
    if (!boost::filesystem::exists(dir)) {
      boost::filesystem::create_directory(dir);
    }
    ostringstream oss; oss << "dat/bg" << n << ".dat";
    boost::filesystem::path p(oss.str());
    if (!boost::filesystem::exists(p)) {
      cout << "BG saved: " << oss.str() << endl;
      pIniter->SaveBackGroundToFile(oss.str());
      return;
    }
    else {
      ++n;
    }
  }
}

void CFLTKPresenter::ProcKeyCode(int nKeyCode) {

  if (nKeyCode == (int)'b') {
    SaveIniterBG();
  }
  else if (nKeyCode == (int)'r') {
    cout << "reset all!" << endl;
    ResetAllTrack();
  }
  else if (nKeyCode == (int)'t') {
    cout << "toggle track!" << endl;
    ToggleTrackMode();
  }
}

void CTrackerResultViewModel::DrawParticle(CTrackerResultView* pView, const BoostVec &rvParticle, int nType, uint8_t r, uint8_t g, uint8_t b, bool bBold, bool bDrawSpeed) {
  if (nType == 0) {
    _vTemp(0) = rvParticle(0);
    _vTemp(1) = rvParticle(1);
    pView->DrawCylinder(_vTemp, rvParticle(4), 1800, 0, r, g, b, bBold);
    if (bDrawSpeed) {
      _vTemp2(0) = rvParticle(2);
      _vTemp2(1) = rvParticle(3);
      pView->DrawSpeedArow(_vTemp, _vTemp2, 255,255,255);
    }
  }
  else if (nType == 1) {
    _vTemp(0) = rvParticle(0);
    _vTemp(1) = rvParticle(1);
    pView->DrawEllipticCylinder(_vTemp, rvParticle(4), rvParticle(5), 
      FastMath::table_atan2(rvParticle(3), rvParticle(2)), 1800, 0, r, g, b, bBold);
    if (bDrawSpeed) {
      _vTemp2(0) = rvParticle(2);
      _vTemp2(1) = rvParticle(3);
      pView->DrawSpeedArow(_vTemp, _vTemp2, 255,255,255);
    }
  }
  else if (nType == 2) {

    _vTemp(0) = rvParticle(0);
    _vTemp(1) = rvParticle(1);
    pView->DrawCuboid(_vTemp, rvParticle(4), rvParticle(5), 
      FastMath::table_atan2(rvParticle(3), rvParticle(2)), 1800, 0, r, g, b, bBold);
    if (bDrawSpeed) {
      _vTemp2(0) = rvParticle(2);
      _vTemp2(1) = rvParticle(3);
      pView->DrawSpeedArow(_vTemp, _vTemp2, 255,255,255);
    }

  }
  else if (nType == 3) {

    _vTemp(0) = rvParticle(0);
    _vTemp(1) = rvParticle(1);
    pView->DrawPoint(_vTemp, r,g, b, 3, false);
    if (bDrawSpeed) {
      _vTemp2(0) = rvParticle(2);
      _vTemp2(1) = rvParticle(3);
      pView->DrawSpeedArow(_vTemp, _vTemp2, 255,255,255);
    }

  }
}


void CTrackerResultViewModel::DrawBeforeObjects(CTrackerResultView* pView, const boost::shared_ptr<const STrackerResult> &pTrackerResult2, const std::vector<boost::shared_ptr<const CLaserData> > &rvpLaserData) {
  
  const SJPDATrackerResult* pTrackerResult = 
    dynamic_cast<const SJPDATrackerResult*>( pTrackerResult2.get());
  if (!pTrackerResult)  {
    cout << __FUNCTION__ << " something wrong" << endl;
    return;
  }
//  const auto &rConfig = pTrackerResult->_Config;

  const auto &rConfig = pTrackerResult->_Config;

  //regionを描画
  if (!rConfig._vDetectRegion.empty()) {
    for (auto itRegion = rConfig._vDetectRegion.begin(); itRegion != rConfig._vDetectRegion.end(); ++itRegion) {
      pView->DrawRegion(*itRegion, 20, 0, 255, 255);
    }
  }


  static vector<CvScalar> vCol;
  if (vCol.empty()) {
    vCol += 
      CV_RGB(255,255,0), 
      CV_RGB(255,0, 255), 
      CV_RGB(0,255,0), 
      CV_RGB(255,0,0);
  }
//  for (auto it = pTrackerResult->_vpObjects.begin(); it != pTrackerResult->_vTempPFResults.end(); ++it)  {

  //検出領域を描画
  /*
  if (pTrackerResult->_vDetectRegion.size()>=3) {
    for (size_t i=0; i<pTrackerResult->_vDetectRegion.size(); ++i) {
      const BoostVec &r1 = pTrackerResult->_vDetectRegion[i];
      size_t i2 = i+1; if (i2 == pTrackerResult->_vDetectRegion.size()) i2=0;
      const BoostVec &r2 = pTrackerResult->_vDetectRegion[i2];
      pView->DrawLine(r1, r2, 255, 255, 0, 1);
    }
  }
  */

  int n=0;
  for (auto it = pTrackerResult->_vTempPFResults.begin(); it != pTrackerResult->_vTempPFResults.end(); ++it, ++n)  {

    int nType = TypeNameToInt((*it)->_sType);
    int nID = pTrackerResult->_vpObjects[n]->GetID();

    auto &col  = vCol[nID%vCol.size()];


    if (_nParticleDrawMode == 0) {

      if (false) { //Original用
        for (auto itParticle = (*it)->_vParticles.begin(); itParticle != (*it)->_vParticles.end(); ++itParticle) {
          DrawParticle(pView, *itParticle, nType, (uint8_t)col.val[2], (uint8_t)col.val[1], (uint8_t)col.val[0], false);
        }
      }
      else { //Normal用
        for (auto itParticle = (*it)->_vParticles.begin(); itParticle != (*it)->_vParticles.end(); ++itParticle) {
          DrawParticle(pView, *itParticle, nType, 0, 55, 0, false);
        }
      }
    }
    else { //_nParticleDrawMode==2の時もPredictedを描画
      for (auto itParticle = (*it)->_vParticlesPredict.begin(); itParticle != (*it)->_vParticlesPredict.end(); ++itParticle) {
        DrawParticle(pView, *itParticle, nType, 55, 55, 0, false);
      }
    }
  }

  for (auto itLine=pTrackerResult->_vBestHypoDivLines.begin(); itLine!=pTrackerResult->_vBestHypoDivLines.end(); ++itLine) {

    CAngle d1 = _Prop.IndexToAngle((unsigned int)(*itLine));
    double dLen = rvpLaserData.front()->GetProperty()._dMaxRange;
    BoostVec v2(3); v2(0) = dLen*cos(d1); v2(1) = dLen*sin(d1); v2(2) = 0;
    auto pCo = rvpLaserData.front()->GetCo();
    pCo->VectorLocalToGlobal(v2);
    pView->DrawLine(pCo->GetPos(), v2, 255, 0, 0, 1);
  }
}


void CTrackerResultViewModel::DrawAfterObjects(CTrackerResultView* pView, const boost::shared_ptr<const STrackerResult> &pTrackerResult2, const std::vector<boost::shared_ptr<const CLaserData> > &rvpLaserData) {

  const SJPDATrackerResult* pTrackerResult = 
    dynamic_cast<const SJPDATrackerResult*>( pTrackerResult2.get());
  if (!pTrackerResult)  {
    cout << __FUNCTION__ << " something wrong" << endl;
    return;
  }
  
  for (auto it = pTrackerResult->_vExtractedPoints.begin(); it!=pTrackerResult->_vExtractedPoints.end(); ++it) {
    pView->DrawPoint( (*it)->GetWorldVec(), 0, 255, 255, 9, false);
  } 
  
  const auto &rvPointStatus = pTrackerResult->_vPointStatus;
  const auto &rvpPoints = rvpLaserData.at(0)->GetPoints();

  for (size_t i=0; i<rvPointStatus.size(); ++i) {

    if (rvPointStatus[i] == 1) {
      pView->DrawPoint( rvpPoints[i]->GetWorldVec(), 0, 255, 255, 9, false);
    }
    else if (rvPointStatus[i] == 2) {
      pView->DrawPoint( rvpPoints[i]->GetWorldVec(), 255, 0, 0, 9, false);
    }
  }

  if ((!pTrackerResult->_vTempPFResults.empty()) && (_nParticleDrawMode == 2)) {
    if (_nDebugPFNum < 0) {
      _nDebugPFNum = (int)(pTrackerResult->_vTempPFResults.size()-1);
    }
    else if (_nDebugPFNum >= (int)pTrackerResult->_vTempPFResults.size()) {
      _nDebugPFNum = 0;
    }
    const auto &pPFResult = pTrackerResult->_vTempPFResults.at(_nDebugPFNum);
    if (_nDebugParticleNum < 0) {
      _nDebugParticleNum = (int)(pPFResult->_vParticlesPredict.size())-1;
    }
    else if (_nDebugParticleNum >= (int)(pPFResult->_vParticlesPredict.size())) {
      _nDebugParticleNum = 0;
    }
    int nType = TypeNameToInt(pPFResult->_sType);
    const auto &rvPos = pPFResult->_vParticlesPredict.at(_nDebugParticleNum);
    DrawParticle(pView, rvPos, nType, 255, 0, 0, true);

    double dLikelihood = pPFResult->_vdLikelihood.at(_nDebugParticleNum);
    ostringstream oss;
    oss << "PFID:" << pPFResult->_nClusterID << " PNum:" << _nDebugParticleNum << " Li:";
    oss << setw(2) << dLikelihood;

    _vTemp(0) = rvPos(0);
    _vTemp(1) = rvPos(1);
    pView->DrawCaption(_vTemp, oss.str(), 255, 0, 0);

    /*

      ostringstream oss; oss << "P" << (*it)->GetID();
      uint8_t r2, g2, b2;
      b2 = 0;
      if (pStatus->_dExistenceRate < 0.5) {
        g2 = (int)round(pStatus->_dExistenceRate*255.0*2);
        r2 = 255;
      }
      else {
        g2 = 255;
        r2 = (int)round(255.0-(pStatus->_dExistenceRate-0.5)*255.0*2);
      }
      pView->DrawCaption(rCurrentPos, oss.str(), r2, g2, b2);
      */
  }

}

int CTrackerResultViewModel::TypeNameToInt(const std::string &rsType) {

  if (rsType == "cylinder") {
    return 0;
  }
  else if (rsType == "ellipsecylinder") {
    return 1;
  }
  else if (rsType == "rectangle") {
    return 2;
  }
  else if (rsType == "point") {
    return 3;
  }
  else {
    cout << "unknown type: " << rsType << endl;
    return -1;
  }
}

void CTrackerResultViewModel::DrawTrackerResult(CTrackerResultView* pView) {

  static vector<CvScalar> vCol;
  if (vCol.empty()) {
    vCol += 
//      CV_RGB(255,255,255),
      CV_RGB(255,255,0), 
      CV_RGB(255,0, 255), 
  //    CV_RGB(0,255,255), 
  //    CV_RGB(0,0,255), 
      CV_RGB(0,255,0), 
      CV_RGB(255,0,0);
  }

  mmtimer mt;
  pView->ClearWindow(0,0,0);
  if (_History.empty()) return;

  bool bFrameChanged = false;
  int nCurrentFrame = (*_History.rbegin())->first->_nFrame;
  if (nCurrentFrame == _nLastFrame) {
    _nRollbackFrame = min(max(0, _nRollbackFrame), ((int)_History.size())-1);
  }
  else {
    //フレーム更新時
    _nRollbackFrame = 0;
    bFrameChanged = true;
  }
  _nLastFrame = nCurrentFrame;

  auto itFirst = _History.rbegin();
  for (int n=0; n<_nRollbackFrame; ++n) {
    ++itFirst;
  }
  const auto &pResult = *itFirst;
  const boost::shared_ptr<const STrackerResult> &pTrackerResult = pResult->first;
  const std::vector<boost::shared_ptr<const CLaserData> > &rvpLaserData = pResult->second;

  if (bFrameChanged && _bChaseLRF) {
    auto pCo = rvpLaserData.front()->GetCo();
    pView->SetViewerCenter(pCo->GetX(), pCo->GetY());
  }

  int nFrame = pTrackerResult->_nFrame;
  //空き領域を描画
  for (auto itLaser = rvpLaserData.begin(); itLaser != rvpLaserData.end(); ++itLaser) {
    pView->DrawFreeSpace(*itLaser,100,100,100);
  }
  //temp
  _Prop = rvpLaserData.begin()->get()->GetProperty();
  DrawBeforeObjects(pView, pTrackerResult, rvpLaserData);

  //ポイント描画
  for (auto itLaser = rvpLaserData.begin(); itLaser != rvpLaserData.end(); ++itLaser) {
    pView->DrawPoints(*itLaser,0,128,128,5);
  }

  DrawAfterObjects(pView, pTrackerResult, rvpLaserData);

  //センサー描画
  for (auto itLaser = rvpLaserData.begin(); itLaser != rvpLaserData.end(); ++itLaser) {
    pView->DrawSensor((*itLaser)->GetCo(), "URG");
  }


  //現在状態, 履歴描画
  for (auto it = pTrackerResult->_vpObjects.begin(); it!= pTrackerResult->_vpObjects.end(); ++it) {
    const auto &Col = vCol[(*it)->GetID()%vCol.size()];

    pView->DrawHistory((*it)->GetStatusLog(), _nHistoryDrawLen, (uint8_t)Col.val[2], (uint8_t)Col.val[1], (uint8_t)Col.val[0]);
    
    if ((*it)->GetType() == "cylinder") {
      const auto* pStatus = (const SCylinderMovingObjectStatus*)((*it)->GetStatus().get());
      const BoostVec &rCurrentPos = pStatus->_vPos;
      const BoostVec &rCurrentVel = pStatus->_vVel;
      double dRadius = pStatus->_dRadius;
      double dMin = pStatus->_dMinHeight;
      double dMax = pStatus->_dMaxHeight;

      pView->DrawCylinder(rCurrentPos, dRadius, dMax, dMin, 128,255,128, true);
      pView->DrawSpeedArow(rCurrentPos, rCurrentVel, 255,255,255);

      ostringstream oss; oss << "P" << (*it)->GetID();
      uint8_t r2, g2, b2;
      b2 = 0;
      if (pStatus->_dExistenceRate < 0.5) {
        g2 = (int)round(pStatus->_dExistenceRate*255.0*2);
        r2 = 255;
      }
      else {
        g2 = 255;
        r2 = (int)round(255.0-(pStatus->_dExistenceRate-0.5)*255.0*2);
      }
      pView->DrawCaption(rCurrentPos, oss.str(), r2, g2, b2);

    }
    else if ((*it)->GetType() == "ellipsecylinder") {
      const auto* pStatus = (const SEllipseCylinderMovingObjectStatus*)((*it)->GetStatus().get());
      const BoostVec &rCurrentPos = pStatus->_vPos;
      const BoostVec &rCurrentVel = pStatus->_vVel;
      double dR1 = pStatus->_dR1;
      double dR2 = pStatus->_dR2;
      double dMin = pStatus->_dMinHeight;
      double dMax = pStatus->_dMaxHeight;

      pView->DrawEllipticCylinder(rCurrentPos, dR1, dR2, FastMath::table_atan2(rCurrentVel(1), rCurrentVel(0)), dMax, dMin, 128,255,128, true);
      pView->DrawSpeedArow(rCurrentPos, rCurrentVel, 255,255,255);
      ostringstream oss; oss << "P" << (*it)->GetID();
      oss << fixed << setprecision(2);
      oss << "(" << pStatus->_dExistenceRate << ")";
      uint8_t r2, g2, b2;
      b2 = 0;
      if (pStatus->_dExistenceRate < 0.5) {
        g2 = (int)round(pStatus->_dExistenceRate*255.0*2);
        r2 = 255;
      }
      else {
        g2 = 255;
        r2 = (int)round(255.0-(pStatus->_dExistenceRate-0.5)*255.0*2);
      }
      pView->DrawCaption(rCurrentPos, oss.str(), r2, g2, b2);
    }
    else if ((*it)->GetType() == "rectangle") {

      const auto* pStatus = (const SCuboidMovingObjectStatus*)((*it)->GetStatus().get());
      const BoostVec &rCurrentPos = pStatus->_vPos;
      const BoostVec &rCurrentVel = pStatus->_vVel;
      double dMin = pStatus->_dMinHeight;
      double dMax = pStatus->_dMaxHeight;

      pView->DrawCuboid(rCurrentPos, pStatus->_dLength, pStatus->_dWidth, FastMath::table_atan2(rCurrentVel(1), rCurrentVel(0)), dMax, dMin, 128,255,128, true);

      pView->DrawSpeedArow(rCurrentPos, rCurrentVel, 255,255,255);
      ostringstream oss; oss << "C" << (*it)->GetID();
      uint8_t r2, g2, b2;
      b2 = 0;
      if (pStatus->_dExistenceRate < 0.5) {
        g2 = (int)round(pStatus->_dExistenceRate*255.0*2);
        r2 = 255;
      }
      else {
        g2 = 255;
        r2 = (int)round(255.0-(pStatus->_dExistenceRate-0.5)*255.0*2);
      }
      pView->DrawCaption(rCurrentPos, oss.str(), r2, g2, b2);
    }
    else {

      const auto* pStatus = ((*it)->GetStatus().get());
      const BoostVec &rCurrentPos = pStatus->_vPos;
      const BoostVec &rCurrentVel = pStatus->_vVel;
      static vector<CvScalar> vCol;
      if (vCol.empty()) {
        vCol += 
          CV_RGB(255,255,0), 
          CV_RGB(255,0, 255), 
          CV_RGB(0,255,0), 
          CV_RGB(255,0,0);
      }
      auto &rCol = vCol[(*it)->GetID()%vCol.size()];

//      pView->DrawPoint(rCurrentPos, 128, 255, 128, 20, true);
      pView->DrawPoint(rCurrentPos, (uint8_t)rCol.val[2], (uint8_t)rCol.val[1], (uint8_t)rCol.val[0], 20, true);
      pView->DrawSpeedArow(rCurrentPos, rCurrentVel, 255,255,255);
      ostringstream oss; oss << "X" << (*it)->GetID();
      uint8_t r2, g2, b2;
      b2 = 0;
      if (pStatus->_dExistenceRate < 0.5) {
        g2 = (int)round(pStatus->_dExistenceRate*255.0*2);
        r2 = 255;
      }
      else {
        g2 = 255;
        r2 = (int)round(255.0-(pStatus->_dExistenceRate-0.5)*255.0*2);
      }
      pView->DrawCaption(rCurrentPos, oss.str(), r2, g2, b2);
    }

  }



  ostringstream oss;
  //文字情報を描画
  oss.setf(ios::fixed, ios::floatfield);
  oss << "Frame: " << pTrackerResult->_nFrame;
  if (_nRollbackFrame != 0) {
    oss << " (RollBack=" << _nRollbackFrame << ") ";
  }
  oss << setprecision(2) << " Time:" << pTrackerResult->_dCurrentTime;
//  oss << " PersonNum: " << pTrackerResult->_vpObjects.size() << "/" << pTrackerResult->_Config._nMaxTargetNum;
  oss << " PersonNum: " << pTrackerResult->_vpObjects.size();
  pView->DrawMessage(oss.str(), 255, 255, 255);


  ostringstream oss2;
  oss2 << setprecision(4);
  oss2 <<  "StepTime: " << pTrackerResult->_dStepTime;
  oss2 << " ProcTime:" << pTrackerResult->_dProcTime;
  oss2 << " DrawTime:" << mt.elapsed();
  pView->DrawMessage(oss2.str(), 255, 255, 255);

//  const boost::shared_ptr<const STrackerResult> &pTrackerResult = pResult.first;
//  const std::vector<boost::shared_ptr<const CLaserData> > &rvpLaserData = pResult.second;

  if (_bChaseLRF) {
    pView->DrawMessage("LRF Chase Mode", 255, 255 ,255);
  }

  pView->ShowWindow();
}

void CFLTKWindowView::DrawHistory(const std::vector<boost::shared_ptr<const SMovingObjectStatus> > & rHistory, size_t nDrawNum, uint8_t r, uint8_t g, uint8_t b) {
  if (rHistory.size() < 2) return;
  auto Col = CV_RGB(r,g,b);

  int nNum = (int)min(nDrawNum, rHistory.size()-1);

  for (int i=0; i<nNum; ++i) {
    //履歴は後ろから始まる
    const auto &r1 = rHistory[rHistory.size()-1-i]->_vPos;
    const auto &r2 = rHistory[rHistory.size()-2-i]->_vPos;
    _pLRFViewer->DrawLine(r1(0), r1(1), r2(0), r2(1), Col, 2);
  }

}


void CFLTKWindowView::DrawHistory(const boost::circular_buffer<boost::shared_ptr<const SMovingObjectStatus> > & rHistory, size_t nDrawNum, uint8_t r, uint8_t g, uint8_t b) {
  if (rHistory.size() < 2) return;
  auto Col = CV_RGB(r,g,b);

  int nNum = (int)min(nDrawNum, rHistory.size()-1);

  for (int i=0; i<nNum; ++i) {
    //履歴は後ろから始まる
    const auto &r1 = rHistory[rHistory.size()-1-i]->_vPos;
    const auto &r2 = rHistory[rHistory.size()-2-i]->_vPos;
    _pLRFViewer->DrawLine(r1(0), r1(1), r2(0), r2(1), Col, 2);
  }

}


void CFLTKWindowView::DrawFreeSpace(const boost::shared_ptr<const CLaserData> &pLaserData, uint8_t r, uint8_t g, uint8_t b) {

  auto Col = CV_RGB(r,g,b);
  _pLRFViewer->DrawFreeSpace(pLaserData, Col);
}

void CFLTKWindowView::DrawPoints(const boost::shared_ptr<const CLaserData> &pLaserData, uint8_t r, uint8_t g, uint8_t b, int nSize) {

  for (auto p = pLaserData->GetPoints().begin(); p != pLaserData->GetPoints().end(); ++p) {
    _pLRFViewer->DrawCross( (*p)->GetX(), (*p)->GetY(), CV_RGB(r,g,b), nSize);
  }
}

void CFLTKWindowView::DrawPoint(const BoostVec &vPos, uint8_t r, uint8_t g, uint8_t b, int nSize, bool bBold) {

  if (bBold) {
    _pLRFViewer->DrawCross( vPos(0), vPos(1), CV_RGB(255,255,255), nSize+3, 5);
    _pLRFViewer->DrawCross( vPos(0), vPos(1), CV_RGB(r,g,b), nSize, 3);
  }
  else {
    int nWidth = 1;
    _pLRFViewer->DrawCross( vPos(0), vPos(1), CV_RGB(r,g,b), nSize, nWidth);
  }
}

void CFLTKWindowView::DrawLine(const BoostVec &v1, const BoostVec &v2, uint8_t r, uint8_t g, uint8_t b, int nWidth) {

  _pLRFViewer->DrawLine(v1(0), v1(1), v2(0), v2(1), CV_RGB(r,g,b), nWidth);
}


void CFLTKWindowView::ClearWindow(uint8_t r, uint8_t g, uint8_t b) {
  
  _pLRFViewer->ResetViewer(CV_RGB(r,b,g));

  _nMsgCount = 0;
}

void CFLTKWindowView::DrawCylinder(const BoostVec &rCurrentPos, double dRad, double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold) {
  int nWidth = 1;
  if (bBold) nWidth = 2;

  _pLRFViewer->DrawCircle(rCurrentPos(0), rCurrentPos(1), CV_RGB(r,g,b), dRad, nWidth);
}


void CFLTKWindowView::DrawEllipticCylinder(const BoostVec &rPos, double dR1, double dR2, CAngle dAngle,  double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold) {
  int nWidth = 1;
  if (bBold) nWidth = 2;

  _pLRFViewer->DrawEllipse(rPos(0), rPos(1), dR1, dR2, dAngle, CV_RGB(r,g,b), nWidth);
}


void CFLTKWindowView::DrawCuboid(const BoostVec &rPos, double dLength, double dWidth, CAngle dAngle,  double dTop, double dBottom, uint8_t r, uint8_t g, uint8_t b, bool bBold) {

  int nWidth = 1;
  if (bBold) nWidth = 2;

  _pLRFViewer->DrawSquare(rPos(0), rPos(1), dAngle.get(), dLength, dWidth, CV_RGB(r,g,b), nWidth);
}



void CFLTKWindowView::DrawSpeedArow(const BoostVec &rCurrentPos, const BoostVec &rCurrentVel, uint8_t r, uint8_t g, uint8_t b) {

  double dx = rCurrentPos(0);
  double dy = rCurrentPos(1);
  double dVel = _hypot(rCurrentVel(0), rCurrentVel(1));
  double dAngle = atan2(rCurrentVel(1), rCurrentVel(0));
  double dVx = dVel*cos(dAngle);
  double dVy = dVel*sin(dAngle);
  double dAngle2 = dAngle-Deg2Rad(150);
  double dVx2 = 0.3*dVel*cos(dAngle2);
  double dVy2 = 0.3*dVel*sin(dAngle2);
  double dLen = 1000;
  _pLRFViewer->DrawLine(dx,dy,dx+dVx,dy+dVy,CV_RGB(r,g,b), 2);
  _pLRFViewer->DrawLine(dx+dVx,dy+dVy, dx+dVx+dVx2, dy+dVy+dVy2,CV_RGB(r,g,b), 2);
}

void CFLTKWindowView::DrawSensor(const boost::shared_ptr<const CCascadedCoords> &pLRFCo, const std::string &rSensor) {

  _pLRFViewer->DrawCircle(pLRFCo->GetX(), pLRFCo->GetY(), CV_RGB(255,255,255), 300, -1);
  double dX2 = pLRFCo->GetX() + 500*cos(pLRFCo->GetYaw());
  double dY2 = pLRFCo->GetY() + 500*sin(pLRFCo->GetYaw());

  _pLRFViewer->DrawLine(pLRFCo->GetX(), pLRFCo->GetY(), dX2, dY2, CV_RGB(255,255,255), 3);
}

void CFLTKWindowView::ShowWindow() {
  redraw();
}

void CFLTKWindowView::DrawMessage(const std::string &rsMsg, uint8_t r, uint8_t g, uint8_t b) {

  _pLRFViewer->DrawString(rsMsg, 5, 40+_nMsgCount*20, CV_RGB(r,g,b), false);
  ++_nMsgCount;
}

void CFLTKWindowView::DrawCaption(const BoostVec &vPos, const std::string &rsMsg, uint8_t r, uint8_t g, uint8_t b) {

  double dx = vPos(0);
  double dy = vPos(1);
  double dLen = 1000;

  _pLRFViewer->DrawLine(dx, dy, dx+dLen, dy-dLen, CV_RGB(r,g,b));
  _pLRFViewer->DrawString(rsMsg, dx+dLen, dy-dLen, CV_RGB(r,g,b));
}


void CFLTKWindowView::Update() {

  CTrackerResultViewModel *pViewModel = dynamic_cast<CTrackerResultViewModel *>(_pViewModel.get());
  if (!pViewModel) {
    cout << __FUNCTION__ <<  " somthing wrong: no viewmodel" << endl;
    return;
  }

  double d1 = GetSliderVal()*1000;
  _pLRFViewer->SetWidthLength(d1);
  pViewModel->DrawTrackerResult(this);
}

void CFLTKWindowView::DrawRegion(const CPolygonalRegion &rRegion, double dIntervalPixel, uint8_t r, uint8_t g, uint8_t b) {
  vector<BoostVec> vPoints;
  rRegion.GetPoints(vPoints);
  if (vPoints.size() < 2) return;
  auto Col = CV_RGB(r,g,b);

  for (size_t i=0; i<vPoints.size(); ++i) {
    size_t i2=i+1; if (i2==vPoints.size()) i2 = 0;
    _pLRFViewer->DrawLine( vPoints[i](0), vPoints[i](1), vPoints[i2](0), vPoints[i2](1), Col, 2);
  }

}


CFLTKWindowView::CFLTKWindowView(
  boost::shared_ptr<CMOTrackerViewModel> pViewModel, 
  boost::shared_ptr<CCvLRFViewer> pViewer
  )
  : CTrackerResultView(pViewModel), fltk::Window(
                    pViewer->GetBufImage()->width,
                    pViewer->GetBufImage()->height+50,
                    pViewer->GetWindowName().c_str())
{
  _BufImage = nullptr;
  _pLRFViewer = pViewer;
  _nMsgCount = 0;
  Init();
}


CFLTKWindowView::~CFLTKWindowView() {

  cvReleaseImage(&_BufImage);
}

int CFLTKWindowView::handle(int nEvent) {


  int nX = fltk::event_x();
  int nY = fltk::event_y();
  bool b1 = fltk::event_state(fltk::BUTTON1);
  bool b2 = fltk::event_state(fltk::BUTTON2);
  bool b3 = fltk::event_state(fltk::BUTTON3);

  auto* pVM = dynamic_cast<CTrackerResultViewModel*>(_pViewModel.get());
  if (!pVM) {
    cout << __FUNCTION__ << "something wrong" << endl;
    return 0;
  }

  switch(nEvent) {

  case fltk::KEY:
  {
    int nKey = fltk::event_key();
    pVM->KeyPressed(this, nKey);
    break;
  }
  case fltk::PUSH:
  {
    pVM->MousePressed(this, b1, b2, b3, nX, nY);
    break;  
  }
  case fltk::MOVE:
  {
    pVM->MouseMoved(this, b1, b2, b3, nX, nY);
    break;  
  }
  case fltk::RELEASE:
  {
    pVM->MouseReleased(this, b1, b2, b3, nX, nY);
    break;  
  }
    default:
      break;
  }


  return fltk::Window::handle(nEvent);
}


void CFLTKWindowView::ResetViewerRange() {
  //reset
  _pLRFViewer->SetCenterPos(_dOrgXCenter, _dOrgYCenter);
  _pLRFViewer->SetWidthLength(_dOrgWidthLength);

  _pSlider->value(_pLRFViewer->GetWidthLength()/1000.0);
}


void CFLTKWindowView::StartDrag() {

  if (fltk::event_state(fltk::BUTTON1) && fltk::event_state(fltk::BUTTON3) ) {
    ResetViewerRange();
  }
  else if (fltk::event_state(fltk::BUTTON1)) {

    boost::tie(_dCurX, _dCurY) = _pLRFViewer->GetGlobalPos(fltk::event_x(), fltk::event_y());

    _XStart = fltk::event_x();
    _YStart = fltk::event_y();
    _CurrentState.nState = SDragState::eStart;
    _CurrentState.dStartX = fltk::event_x();
    _CurrentState.dStartY = fltk::event_y();
    _CurrentState.dLastX = _CurrentState.dStartX;
    _CurrentState.dLastY = _CurrentState.dStartY;
    _CurrentState.dCurrentX = _CurrentState.dStartX;
    _CurrentState.dCurrentY = _CurrentState.dStartY;
  }
}


void CFLTKWindowView::MoveDrag() {


  if (fltk::event_state(fltk::BUTTON1)) {
    _CurrentState.nState = SDragState::eDrag;
    _CurrentState.dLastX = _CurrentState.dCurrentX;
    _CurrentState.dLastY = _CurrentState.dCurrentY;
    _CurrentState.dCurrentX = fltk::event_x();
    _CurrentState.dCurrentY = fltk::event_y();
  }
  else {
    _CurrentState.nState = SDragState::eNothing;
  }
}

void CFLTKWindowView::EndDrag() {
  _CurrentState.nState = SDragState::eEnd;
}


void ResetButtonPushed2(fltk::Button *o, void *p) {

  CFLTKWindowView* pWindow = (CFLTKWindowView *)p;
  pWindow->ResetViewerRange();
}




void CFLTKWindowView::draw() {

  using namespace fltk;

  fltk::Window::draw();
  IplImage *_pCvImage = _pLRFViewer->GetBufImage();
  fltk::Rectangle Rect(0, 0, _pCvImage->width, _pCvImage->height);

//  fltk::drawimage((const uchar*)_pCvImage->imageData, fltk::BGR, Rect); //BGRは公開版にはない
  cvCvtColor(_pCvImage, _BufImage, CV_BGR2RGB); //無駄だが1ms程度で終わる気がする
  fltk::drawimage((const uchar*)_BufImage->imageData, fltk::RGB, Rect); 

  if ((_dCurX != DBL_MAX) || (_dCurY != DBL_MAX)) {
    setcolor(WHITE);
    setfont(labelfont(), labelsize());
    ostringstream oss; oss << fixed << setprecision(1);
    oss << "Clicked Pos: (" << _dCurX << "," << _dCurY << ")";
    drawtext(oss.str().c_str(), 10.0f, h()-60.0f);
  }
}


void CFLTKWindowView::Init() {
  set_vertical();
//      shortcut(0xff1b);
  begin();
//      new fltk::ValueSlider(184, 640, 362, 35);
  _pSlider =  new fltk::ValueSlider(100, _pLRFViewer->GetHeight()+5, _pLRFViewer->GetWidth()-110, 35);
  _pSlider->range(0.5, 100);
  _pSlider->value(_pLRFViewer->GetWidthLength()/1000.0);

  _pResetButton = new fltk::Button(10, _pLRFViewer->GetHeight()+5, 80, 35, "Reset View");
  _pResetButton->callback((fltk::Callback*)ResetButtonPushed2, this);

  end();

  _dOrgXCenter = _pLRFViewer->GetXCenter();
  _dOrgYCenter = _pLRFViewer->GetYCenter();
  _dOrgWidthLength = _pLRFViewer->GetWidthLength();
  _dCurX = DBL_MAX;
  _dCurY = DBL_MAX;

  _CurrentState.nState = SDragState::eNothing;
  cvReleaseImage(&_BufImage);
  _BufImage = cvCreateImage(cvSize(_pLRFViewer->GetBufImage()->width,_pLRFViewer->GetBufImage()->height), _pLRFViewer->GetBufImage()->depth, _pLRFViewer->GetBufImage()->nChannels);


}

void CFLTKWindowView::SetViewerCenter(double x, double y) {
  
  _pLRFViewer->SetCenterPos(x,y);

}
void CFLTKWindowView::SaveImage(const std::string &rsFileName) {
  if (!cvSaveImage(rsFileName.c_str(), GetViewer()->GetBufImage())) {
    throw std::logic_error("CFLTKWindowView::SaveImage failed");
  }
}


void ChangeDoutSetting() {
  size_t n=dout.GetMode();
  ++n;
#ifdef _MSC_VER
  if (n>3) n=0;
#else
  if (n>2) n=0;
#endif
  dout.SetMode(n);
  if (n==0) cout << endl << "dout set noshow" << endl;
  else if (n==2) cout << endl << "dout set printf" << endl;
#ifdef _MSC_VER
  else if (n==1) cout << endl << "dout set OutputDebugString" << endl;
#endif
  else cout << endl << "dout set dout.txt" << endl;
}


void CTrackerResultViewModel::KeyPressed(CTrackerResultView* pView, int nKeyCode) {

//  cout << "key pressed : " << nKeyCode << "/" << (char)nKeyCode << endl;
  if (nKeyCode == ' ') {
    _pPresenter->ToggleWaitMode();
  }
  else if (nKeyCode == (int)'a') {
    _pPresenter->ProcOneFrame();
  }
  else if (nKeyCode == (int)'d') {
    ChangeDoutSetting();
  }
  else if (nKeyCode == (int)',') {
    if (_pPresenter->IsLogMode()) {
      ++_nRollbackFrame;
    }
  }
  else if (nKeyCode == (int)'.') {
    if (_pPresenter->IsLogMode()) {
      --_nRollbackFrame;
    }
  }
  else if (nKeyCode == (int)'c') {
    SetChaseLRF(!GetChaseLRF());
  }
  else if (nKeyCode == (int)'s') {
    if (!boost::filesystem::exists("figure")) {
      boost::filesystem::create_directory("figure");
    }
    ostringstream oss;
    string sTime =  boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
    oss << "figure" <<  "/" << sTime << ".png";
    try {
      pView->SaveImage(oss.str());
      cout << "Saved: " << oss.str() << endl;
    }
    catch (std::exception &e) {
      cout << e.what() << endl;
    }
  }
  else if (nKeyCode == 'p') {
    ToggleParticleDrawMode();
  }
  else if (nKeyCode == '[') {
    --_nDebugParticleNum;
  }
  else if (nKeyCode == ']') {
    ++_nDebugParticleNum;
  }
  else if (nKeyCode == 96) { //'@'
    --_nDebugPFNum;
  }
  else if (nKeyCode == 59) { //':'
    ++_nDebugPFNum;
  }

  _pPresenter->ProcKeyCode(nKeyCode);
}

void CTrackerResultViewModel::MousePressed(CTrackerResultView* pView, bool bButton1, bool bButton2, bool bButton3, int nX, int nY) {

  auto *pView2 = (CFLTKWindowView*)pView;
  auto* pPre2  = (CFLTKPresenter*)_pPresenter;

  if (bButton1 && bButton3) {
    pView2->ResetViewerRange();
  }
  else if (bButton1) {
    _dDragStartX = nX;
    _dDragStartY = nY;

    _dDragLastX = _dDragStartX;
    _dDragLastY = _dDragStartY;
    _bDragging = true;

    pView2->SetMouseClickedPos(nX, nY);
  }
}

void CTrackerResultViewModel::MouseMoved(CTrackerResultView* pView, bool bButton1, bool bButton2, bool bButton3, int nX, int nY) {

  auto *pView2 = (CFLTKWindowView*)pView;
  auto* pPre2  = (CFLTKPresenter*)_pPresenter;

  if (bButton1 && _bDragging) {
    double dCurX = nX;
    double dCurY = nY;

    double x1 = _dDragLastX - dCurX;
    double y1 = _dDragLastY - dCurY;
    double dScale = pView2->GetViewer()->GetScale();
    double dXCenter = pView2->GetViewer()->GetXCenter();
    double dYCenter = pView2->GetViewer()->GetYCenter();
    dXCenter -= y1*dScale;
    dYCenter -= x1*dScale;
    pView2->GetViewer()->SetCenterPos(dXCenter, dYCenter);

    _dDragLastX = dCurX;
    _dDragLastY = dCurY;

    pView2->SetMouseClickedPos(nX, nY);
  }
  else {
    _bDragging = false;
  }
}

void CTrackerResultViewModel::MouseReleased(CTrackerResultView* pView, bool bButton1, bool bButton2, bool bButton3, int nX, int nY) {

}

CPolarGridViewer::CPolarGridViewer(
    boost::shared_ptr<const CTrackInitializationUsingPolarGrid> pGrid, 
    const std::string &sWindowName, int nCellSize) 
    :fltk::DoubleBufferWindow(100,100,"grid1")
{
  _pGrid = pGrid;
  _nCellSize = nCellSize;
  _sWindowName = sWindowName;
  label(_sWindowName.c_str());

  int nX = (int)_pGrid->GetArrayXLen();
  int nY = (int)_pGrid->GetArrayYLen();

  resize(nX*_nCellSize, nY*_nCellSize);
  _pGrid->CopyArray(_aArrayCopied);
}

CPolarGridViewer::~CPolarGridViewer () {}


void CPolarGridViewer::DrawMap() {

  _pGrid->CopyArray(_aArrayCopied);
  std::vector<CTrackInitializationUsingPolarGrid::SCurrentStatus> vStatus;
  _pGrid->CopyStatusDebug(vStatus);

  size_t nX = _pGrid->GetArrayXLen();
  size_t nY = _pGrid->GetArrayYLen();

  uchar r=0;
  uchar g=0;
  uchar b=0;

  for (size_t x=0; x<nX; ++x) {
    for (size_t y=0; y<nY; ++y) {
      size_t nPos = nY*x + y;
      const double &d = _aArrayCopied[nPos];
      if (d > 0) {
        r = (uchar)round( d/_pGrid->GetOddsMax() * 255.0);
        g=0;
        b=0;
      }
      else {
        r = (uchar)round( d/_pGrid->GetOddsMin() * 255.0);
        g=b=r;
      }
      fl_rectf(x*_nCellSize, y*_nCellSize, _nCellSize, _nCellSize, r, g, b);
    }
  }
  for (auto it=vStatus.begin(); it!=vStatus.end(); ++it) {
    size_t nPosX, nPosY;
    boost::tie(nPosX, nPosY) = _pGrid->GetArrayXY(it->nLaserPos, it->dLen);

    if (it->nCellStatus == 0) { //更新するパターン
      r=0;g=0;b=255;
    }
    else if (it->nCellStatus == 1) {//遠距離
      r=0;g=255;b=255; 
    }
    else if (it->nCellStatus == 2) {//背景と重なる
      continue;
    }
    else if (it->nCellStatus == 3) {//奥に物体がある
      r=0;g=255;b=0;
    }
    else if (it->nCellStatus == 4) {//奥までFree
      r=180;g=180;b=0;
    }
    else {
      cout << " muge?" << it->nCellStatus << endl;
    }

    fl_rectf(nPosX*_nCellSize, nPosY*_nCellSize, _nCellSize, _nCellSize, r, g, b);
  }
}

void CPolarGridViewer::draw() {

  fl_font(FL_HELVETICA, 16);
  DrawMap();
}



