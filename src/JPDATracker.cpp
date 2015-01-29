#include "StdAfx_MOTracking.h"
#include "JPDATracker.h"
#include "JPDAFilter.h"
#include "JPDAEllipseFilter.h"
#include "JPDARectangleFilter.h"
#include "TrackInitializationUsingPolarGrid.h"
#include "TrackInitializationUsingGrid.h"
#include "TrackInitializationPassThrough.h"
#include "FastMath.h"
#include "ColorCout.h"
//#include "LibSVM.h"
#include "CLine.h"
//#include "SVMDataProcessing.h"
//#include "SVMMOClassification.h"
#include "SimpleClassification.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/multi_array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/assign.hpp>
#include <boost/scope_exit.hpp>
using namespace boost::assign;
using namespace boost::numeric;
using namespace boost::posix_time;
using namespace std;

//#define MEASURE_TIME
bool CJPDATracker::SPFCluster::s_bUseJIPDA = false;
bool CJPDATracker::CBetaCalculateHelper::s_bUseJIPDA = false;

CJPDATracker::CJPDATracker(const SJPDATrackerConfig &rConfig) {

  _CurrentConfig = rConfig;
  _nCurrentFrame = 0;

  _bUseEllipse = true;
  _bPerformTracking = true;
  _nParticleNum = 256;
  _dIntervalStandardLen = 10000;

  _bUseDynamicProgramming = true;
  _bUseJIPDA = true;
  _bCalcBestHypoInDPMode = false;

  _dRemoveExistanceRateThr = 0.001;
  _dRemoveDisparsionThr = 800;
  
  _bPredictMT=false;
  _dMinClusterDivideThr = 500;
}

CJPDATracker::~CJPDATracker(void) {}

void CJPDATracker::Initialize() {

  bool bCriticalError = true;
  //Initializeに失敗したらプログラム終了
  BOOST_SCOPE_EXIT((&_bCriticalError)(&bCriticalError)) {
      _bCriticalError = bCriticalError;
  } BOOST_SCOPE_EXIT_END

  _nLRFPointNum = _pCurrentLaserData->GetProperty()._nElemNum;

  if (_CurrentConfig._nIniterType == 0) {
    CTrackInitializationUsingGrid *pInit2 = new CTrackInitializationUsingGrid();
    if (!_CurrentConfig._vsLoadGridFileNames.empty() && (_CurrentConfig._vsLoadGridFileNames.at(0) != "")){
      pInit2->Initialize(_CurrentConfig._vsLoadGridFileNames.at(0));
    }
    else {
      const auto &rRange = _CurrentConfig._vGridRange.at(0);
      double dXMax = max(rRange.first(0), rRange.second(0)); 
      double dXMin = min(rRange.first(0), rRange.second(0));
      double dYMax = max(rRange.first(1), rRange.second(1));
      double dYMin = min(rRange.first(1), rRange.second(1));
      pInit2->Initialize(dXMax, dXMin, dYMax, dYMin, _CurrentConfig._dGridSize);
    }
    pInit2->SetObstacleThrLRF(_CurrentConfig._dObsThr);
    pInit2->SetFreeThrLRF(_CurrentConfig._dFreeThr);
    pInit2->SetOddsMinMax(_CurrentConfig._dLRFOddsMin, _CurrentConfig._dLRFOddsMax);
    pInit2->SetOdds(_CurrentConfig._dLRFObsOdds, _CurrentConfig._dLRFFreeOdds);
    _pInitializer = boost::shared_ptr<CTrackInitializationWithProperty>(pInit2);
  }
  else if  (_CurrentConfig._nIniterType == 1) {
    auto *pInit = new 
      CTrackInitializationUsingPolarGrid(_CurrentConfig._dPolarGridMaxLen, _CurrentConfig._dGridSize, _nLRFPointNum, 2);
    pInit->SetOdds(_CurrentConfig._dLRFObsOdds, _CurrentConfig._dLRFFreeOdds);
    pInit->SetStatusThr(_CurrentConfig._dObsThr, _CurrentConfig._dFreeThr);
    pInit->SetOddsMinMax(_CurrentConfig._dLRFOddsMin, _CurrentConfig._dLRFOddsMax);
    _pInitializer = boost::shared_ptr<CTrackInitializationWithProperty>(pInit);
    if (!_CurrentConfig._vsLoadGridFileNames.empty() 
      && (_CurrentConfig._vsLoadGridFileNames.at(0) != ""))
    {
      _pInitializer->LoadBackGroundFromFile(_CurrentConfig._vsLoadGridFileNames.at(0));
    }
  }
  else if  (_CurrentConfig._nIniterType == 2) {
    _pInitializer.reset(new CTrackInitializationPassThrough());
  }
  else {
    ostringstream oss;
    oss << __FUNCTION__ << " unknown Initer type: " << _CurrentConfig._nIniterType;
    throw logic_error(oss.str().c_str());
  }

  _pInitializer->SetUpdateBackGround(_CurrentConfig._bUpdateGrid);

  vector<std::pair<size_t, CPolygonalRegion> > vRegions;
  for (size_t i=0; i<_CurrentConfig._vDetectRegion.size(); ++i) {
    vRegions.push_back(make_pair((size_t)0, _CurrentConfig._vDetectRegion[i]));
  }
  _pInitializer->SetValidRegion(vRegions);
  _dStepTime = 0;

  SetUseJIPDA(_CurrentConfig._bUseJIPDA);
  SetUseDynamicProgramming(_CurrentConfig._bUseDynamicProgramming);
  CJPDAFilter::SetUseRemovedHypoInWeighting(_CurrentConfig._bUseRemovedHypoInWeighting);

  _dRemoveDisparsionThr = _CurrentConfig._dRemoveDisparsionThr;
  _dRemoveExistanceRateThr = _CurrentConfig._dRemoveExistanceRateThr;
  _dIntervalStandardLen = _CurrentConfig._dIntervalStandardLen;

  if (_CurrentConfig._bDumpSVMData) {
//    _pSVMDataProcessing.reset(new CSVMDataProcessing());
  }
  if (_CurrentConfig._bWriteEventLog) {
    string sDirName = "event_log";
    if (!boost::filesystem::exists(sDirName)) {
      boost::filesystem::create_directory(sDirName);
    }
    ostringstream oss;
    oss << sDirName << "/" << to_iso_string(microsec_clock::local_time()) << ".log";
    cout << "filename: " << oss.str() << endl;
    _Logger.SetFile(oss.str());

  }

  CSimpleClassification *pClassifier = new CSimpleClassification();
  _pClassifier.reset(pClassifier);
  /*
  CSVMMOClassification *pClassifier = new CSVMMOClassification;
  pClassifier->OpenSVMDirectory(_CurrentConfig._sFeatureDir);
  */

  bCriticalError = false;
}


void CJPDATracker::SetUseJIPDA(bool b) {
  _bUseJIPDA = b;
  SPFCluster::SetUseJIPDA(b);
  CBetaCalculateHelper::SetUseJIPDA(b);
}

bool CJPDATracker::PrepareTracking(const LaserDataBuffer &rvLasers) {

  if (_nCurrentFrame == 0) {
    _pBeforeLaserData = _pCurrentLaserData;
    _CurrentBuffer = rvLasers;
    _pCurrentLaserData = rvLasers.back().front();
    Initialize();
  }
  else {
    double dTimeDiff = rvLasers.back().front()->GetTime() - _pCurrentLaserData->GetTime();
    if (dTimeDiff < _CurrentConfig._dFrameUpdateMinTime) {
      return false;
    }
    _pBeforeLaserData = _pCurrentLaserData;
    _CurrentBuffer = rvLasers;
    _pCurrentLaserData = rvLasers.back().front();

    _dStepTime = _pCurrentLaserData->GetTime() - _pBeforeLaserData->GetTime();

    if (_CurrentConfig._bUpdateGrid && ((_CurrentConfig._nMaxUpdateGridFrame < 0) ||  (_CurrentConfig._nMaxUpdateGridFrame > _nCurrentFrame))) {
      _pInitializer->SetUpdateBackGround(true);
    }
    else {
      _pInitializer->SetUpdateBackGround(false);
    }
    _pInitializer->SetUpdateBackGround(_CurrentConfig._bUpdateGrid);
  }
  _CurrentFilterData._pLaserData = _pCurrentLaserData;
  return true;
}

void CJPDATracker::ExtractMovingPoints() {

  bool _bRemoveChuukanten = true;

  _vExtractedIDs.clear();
  _vExtractedPoints.clear();
  _vPointStatus.resize(_pCurrentLaserData->GetProperty()._nElemNum);
  std::fill(_vPointStatus.begin(), _vPointStatus.end(), 0);

  std::vector<int> vExtractedIDs2;
  _pInitializer->Update(_CurrentBuffer, vExtractedIDs2);
  for (auto it=vExtractedIDs2.begin(); it!=vExtractedIDs2.end(); ++it) {
    _vPointStatus[*it] = 1;
  }

  if (_bRemoveChuukanten) {
    const auto &rvRawData = _pCurrentLaserData->GetRawData();
    vector<int> vnFound;
    double _dDistThrMax = _CurrentConfig._dRemoveIntervalDistThrMax;
    double _dDistThrMin = _CurrentConfig._dRemoveIntervalDistThrMin;
    double _dDistMulti = _CurrentConfig._dRemoveIntervalDistMulti;

    for (size_t i=0; i<rvRawData.size()-4; ++i) {
      if ( (_vPointStatus[i]==0) &&  (_vPointStatus[i+1]==1) &&  (_vPointStatus[i+2]==1) &&  (_vPointStatus[i+3]==1)) {
        //BG FG FG FG
        if ( (rvRawData[i  ] > rvRawData[i+1]) && (rvRawData[i+1] > rvRawData[i+2]) &&
              (rvRawData[i+1] > rvRawData[i+2]) && (rvRawData[i+2] > rvRawData[i+3])) 
        {
          double dDistDiff = rvRawData[i]-rvRawData[i+3];
          if ((_dDistThrMin < dDistDiff) && (dDistDiff < _dDistThrMax)) {

            if ( (rvRawData[i  ]-rvRawData[i+2] > dDistDiff*_dDistMulti) && 
                  (rvRawData[i+2]-rvRawData[i+3] > dDistDiff*_dDistMulti) )
            {
              vnFound.push_back(i+2);
              vnFound.push_back(i+1);
            }
            else if ( (rvRawData[i  ]-rvRawData[i+1] > dDistDiff*_dDistMulti) && 
                  (rvRawData[i+1]-rvRawData[i+3] > dDistDiff*_dDistMulti) ) 
            {
              vnFound.push_back(i+2);
              vnFound.push_back(i+1);
            }
            else {
            }
          }
        }
      }
      if ( (_vPointStatus[i]==1) &&  (_vPointStatus[i+1]==1) &&  (_vPointStatus[i+2]==1) &&  (_vPointStatus[i+3]==0)) {
        //FG FG FG BG
        if ( (rvRawData[i  ] < rvRawData[i+1]) && (rvRawData[i+1] < rvRawData[i+2]) &&
              (rvRawData[i+1] < rvRawData[i+2]) && (rvRawData[i+2] < rvRawData[i+3])) 
        {
          double dDistDiff = rvRawData[i+3]-rvRawData[i];
          if ((_dDistThrMin < dDistDiff) && (dDistDiff < _dDistThrMax)) {

            if ( (rvRawData[i+2]-rvRawData[i  ] > dDistDiff*_dDistMulti) &&
                  (rvRawData[i+3]-rvRawData[i+2] > dDistDiff*_dDistMulti) ) 
            {
              vnFound.push_back(i+2);
              vnFound.push_back(i+1);
            }
            else if ( (rvRawData[i+1]-rvRawData[i  ] > dDistDiff*_dDistMulti) &&
                  (rvRawData[i+3]-rvRawData[i+1] > dDistDiff*_dDistMulti) ) 
            {
              vnFound.push_back(i+2);
              vnFound.push_back(i+1);
            }
            else {
            }
          }
        }
      }
    }
    for (auto it=vnFound.begin(); it!=vnFound.end(); ++it) {
      _vPointStatus[*it] = 2;
    }
    int nCnt0 = 0;
    int nCnt1 = 0;
    int nCnt2 = 0;
    for (size_t n=0; n<_vPointStatus.size(); ++n) {
      if (_vPointStatus[n] == 1) {
        _vExtractedIDs.push_back(n);
        ++nCnt1;
      }
      else if (_vPointStatus[n] == 0) {
        ++nCnt0;
      }
      else {
        ++nCnt2;
      }
    }
//    cout << "BG: " << nCnt0 << " FG: " << nCnt1 << " NG: " << nCnt2 << endl;
  }
  else {
    _vExtractedIDs = vExtractedIDs2;
  }
  for (auto it=_vExtractedIDs.begin(); it!=_vExtractedIDs.end(); ++it) {
    _vExtractedPoints.push_back( _pCurrentLaserData->GetPoints()[*it] );
  }
  _CurrentFilterData._vExtractedIDs = _vExtractedIDs;
  _CurrentFilterData._vPointStatus = _vPointStatus;
}

void CJPDATracker::MakeCluster() {

  if (_vExtractedIDs.empty()) return;

  vector<int> _vExtractedIDs2;
  _vExtractedClusters.clear();
  _vClusterGroup.clear();
  
  double dExtractThr = _CurrentConfig._dNewObjPointExtractThr;
  for (size_t i=0; i<_vExtractedIDs.size(); ++i) {
    int n = _vExtractedIDs[i];
    if (_vExtractedPointsScore[n] < dExtractThr) {
      _vExtractedIDs2.push_back(n);
    }
  }
  auto _vTemp = _vExtractedIDs;
  _vExtractedIDs = _vExtractedIDs2;

  if (!_vExtractedIDs.empty()) {

    //まず隣接点どうしをクラスタリング
    vector<CJPDAFilter::RangeInterval> vCluster1;

    int nBegin=_vExtractedIDs.front();
    for (size_t n=0; n<_vExtractedIDs.size()-1; ++n) {
      int nID1 = _vExtractedIDs[n];
      int nID2 = _vExtractedIDs[n+1];
      if (nID2-nID1>1) {
        vCluster1.push_back(hull(nBegin, nID1));
        nBegin = nID2;
      }
    }
    if (_vExtractedIDs.size()!=1) {
      vCluster1.push_back(hull(nBegin, _vExtractedIDs.back()));
    }
    //クラスタ内で距離の大きい場所で分離
    vector<CJPDAFilter::RangeInterval> vCluster2;  
    const auto &rRawData = _pCurrentLaserData->GetRawData();
    const auto &rvPoints = _pCurrentLaserData->GetPoints();
    size_t nClus=0;
    for (auto itCluster=vCluster1.begin(); itCluster!=vCluster1.end(); ++itCluster, ++nClus) {

      int nSize = itCluster->upper() - itCluster->lower();
      if (nSize >= 1) {
        vector<int> vBreakPoints;
        vBreakPoints.push_back(itCluster->lower()-1);
        for (int n = itCluster->lower(); n < itCluster->upper(); ++n) {
          double dDist = abs(rRawData[n] - rRawData[n+1]);
          if (dDist > _dMinClusterDivideThr) {
            vBreakPoints.push_back(n);
          }
        }
        vBreakPoints.push_back(itCluster->upper());
        for (size_t i=0; i<vBreakPoints.size()-1; ++i) {
          vCluster2.push_back(hull(vBreakPoints[i]+1, vBreakPoints[i+1]));
        }
      }
      else {
        vCluster2.push_back(*itCluster);
      }
    }

    for (auto itCluster=vCluster2.begin(); itCluster!=vCluster2.end(); ++itCluster) {
      boost::shared_ptr<SPointCluster> p(new SPointCluster);
      for (int n=itCluster->lower(); n<=itCluster->upper(); ++n) {
        p->_vPoints.push_back(rvPoints.at(n));
        p->_vDistances.push_back(rRawData.at(n));
      }
      CAngle low  = _pCurrentLaserData->GetProperty().IndexToAngle(itCluster->lower());
      CAngle high = _pCurrentLaserData->GetProperty().IndexToAngle(itCluster->upper());
      p->_AngleRange = hull(low, high);
      p->_PointIDRange = hull(itCluster->lower(), itCluster->upper());
      p->_dTotalLen = 0;
      p->_dAveDist = 0;
      for (int n=itCluster->lower(); n<itCluster->upper(); ++n) {
        auto v1 = rvPoints.at(n)->GetLocalVec()-rvPoints.at(n+1)->GetLocalVec();
        double dDist = FastMath::fast_hypot(v1(0), v1(1));
        p->_dTotalLen+= dDist;
        p->_dAveDist += _pCurrentLaserData->GetRawData()[n];
      }
      p->_dAveDist += _pCurrentLaserData->GetRawData()[itCluster->upper()];
      p->_dAveDist /= (itCluster->upper()-itCluster->lower()+1);
      p->_dWidthFromLRF = p->_dAveDist * sin((p->_AngleRange.upper()-p->_AngleRange.lower())/2)*2;
      auto v2 = rvPoints.at(itCluster->upper())->GetLocalVec() - rvPoints.at(itCluster->lower())->GetLocalVec();
      p->_dDistBetweenEdge = FastMath::fast_hypot(v2(0), v2(1));

      _vExtractedClusters.push_back(p);
    }

    double _dClusterLenThr = _CurrentConfig._dClusterLenThr;
    double _dClusterAngleThr = _CurrentConfig._dClusterAngleThr;

    int nAngleThr = (int)round(_dClusterAngleThr/_pCurrentLaserData->GetProperty()._dReso.get_deg());
    double dLenthr2 = _dClusterLenThr*_dClusterLenThr;

    if (_vExtractedClusters.empty()) return;

    _vClusterGroup.push_back(vector<int>());
    _vClusterGroup.back().push_back(0);

    const auto &pLasers =  _pCurrentLaserData->GetPoints();
    for (size_t i=1; i<_vExtractedClusters.size(); ++i) {
      auto &pTarget = _vExtractedClusters[i];
      bool bAdded = false;
      bool bOutofRange = false;

      int jTarget = -1;
      for (int j=_vClusterGroup.size()-1; j>=0; --j) {
        const auto &rC = _vClusterGroup[j];
        for (auto itG = rC.rbegin(); itG != rC.rend(); ++itG) {
          int nBeforeLower = _vExtractedClusters[*itG]->_PointIDRange.lower();
          int nBeforeUpper = _vExtractedClusters[*itG]->_PointIDRange.upper();

          for (int n = pTarget->_PointIDRange.lower(); n <= pTarget->_PointIDRange.upper(); ++n) {
            if (n>nBeforeUpper+nAngleThr) {
              bOutofRange = true;
              goto lend;
            }
            int nFirst = max(n-nAngleThr, nBeforeLower);
            double dX1 = pLasers[n]->GetX();
            double dY1 = pLasers[n]->GetY();
            for (int n2 = nFirst; n2 < nBeforeUpper; ++n2) {
              double dX2 = pLasers[n2]->GetX();
              double dY2 = pLasers[n2]->GetY();
              double dDist = (dX1-dX2)*(dX1-dX2)+(dY1-dY2)*(dY1-dY2);
              if (dDist < dLenthr2) {
                bAdded = true;
                jTarget = j;
                goto lend;
              }
            }
          }
        }
      }

  lend:
      if (bAdded) {
        _vClusterGroup[jTarget].push_back((int)i);
      }
      else {
        _vClusterGroup.push_back(vector<int>());
        _vClusterGroup.back().push_back((int)i);
      }
    }
  }
  _vExtractedIDs = _vTemp;
}



//2次元の外積
double Cross2D(const BoostVec &a1, const BoostVec &a2) {
  return a1(0)*a2(1)-a1(1)*a2(0);
}
bool LinesIntersected(const BoostVec &a1, const BoostVec &a2, const BoostVec &b1, const BoostVec& b2) {
  using boost::numeric::ublas::outer_prod;
  return ( Cross2D(a2-a1, b1-a1) * Cross2D(a2-a1, b2-a1) < 0 ) &&
         ( Cross2D(b2-b1, a1-b1) * Cross2D(b2-b1, a2-b1) < 0 );
}

bool HaveCarsCorrelation(CJPDARectangleFilter* pCar1, CJPDARectangleFilter* pCar2) {

  //step1 線分どうしの交差判定
  CPolygonalRegion Region1;
  pCar1->GetResultPolygonal(Region1);
  vector<BoostVec> vPoints1;
  Region1.GetPoints(vPoints1);

  CPolygonalRegion Region2;
  pCar2->GetResultPolygonal(Region2);
  vector<BoostVec> vPoints2;
  Region1.GetPoints(vPoints2);

  for (size_t i=0; i<vPoints1.size(); ++i) {
    const auto &a1 = vPoints1[i];
    size_t i2 = i+1; if (i2 == vPoints1.size()) i2 = 0;
    const auto &a2 = vPoints1[i2];
    for (size_t j=0; j<vPoints2.size(); ++j) {
      const auto &b1 = vPoints2[j];
      size_t j2 = j+1; if (j2 == vPoints2.size()) j2 = 0;
      const auto &b2 = vPoints2[j2];
      if (LinesIntersected(a1, a2, b1, b2)) {
        return true;
      } 
    }
  }
  //step2 完全に内部に含まれている場合
  bool bIn1 = true;
  for (size_t i=0; i<vPoints1.size(); ++i) {
    if (!Region2.IsPointInner(vPoints1[i](0), vPoints1[i](1))) {
      bIn1 = false;
      break;
    }
  }
  if (bIn1) {
    return true;
  }
  bool bIn2 = true;
  for (size_t i=0; i<vPoints2.size(); ++i) {
    if (!Region1.IsPointInner(vPoints2[i](0), vPoints2[i](1))) {
      bIn2 = false;
      break;
    }
  }
  if (bIn2) {
    return true;
  }
  return false;
}

boost::shared_ptr<CJPDAFilter> CJPDATracker::CheckCollision(boost::shared_ptr<CJPDAFilter> pFilter, std::string &rsErrorMsg) {

  //相関を調べる

  //自動車の場合
  {
    auto pTargetCar = dynamic_cast<CJPDARectangleFilter*>(pFilter.get());
    if (pTargetCar) {
      for (auto it=_vpFilters.begin(); it!=_vpFilters.end(); ++it) {
        if (pTargetCar->GetPFID() == (*it)->GetPFID()) continue;
        auto pCheckingCar = dynamic_cast<CJPDARectangleFilter*>(it->get());
        if (pCheckingCar) {
          if (HaveCarsCorrelation(pTargetCar, pCheckingCar)) {

            return *it;
          }
        }
      }
      return boost::shared_ptr<CJPDAFilter>();
    }
  }

  //歩行者の場合
  {
    auto pTargetPerson = dynamic_cast<CJPDAEllipseFilter*>(pFilter.get());
    if (pTargetPerson) {
      for (auto it=_vpFilters.begin(); it!=_vpFilters.end(); ++it) {
        if (pTargetPerson->GetPFID() == (*it)->GetPFID()) continue;
        auto pCheckingCar = dynamic_cast<CJPDARectangleFilter*>(it->get());
        if (pCheckingCar) {
          CPolygonalRegion Region;
          //領域を拡大
          double dLen = pTargetPerson->GetResult()(5);
          //temp avoid fail
          dLen += 800;
          pCheckingCar->GetResultPolygonal(Region, dLen);
          double dX1 = pTargetPerson->GetResult()(0);
          double dY1 = pTargetPerson->GetResult()(1);
          if (Region.IsPointInner(dX1, dY1)) return *it;
        }
        auto pCheckingPerson = dynamic_cast<CJPDAEllipseFilter*>(it->get());
        if (pCheckingPerson) {
          double dX1 = pTargetPerson->GetResult()(0);
          double dY1 = pTargetPerson->GetResult()(1);
          double dX2 = pCheckingPerson->GetResult()(0);
          double dY2 = pCheckingPerson->GetResult()(1);
          double dR = max(pTargetPerson->GetResult()(5),
                          pCheckingPerson->GetResult()(5));
//          double dR = _CurrentConfig._dInitMinDistFromOtherThrPerson;
          double dDist = (dX1-dX2)*(dX1-dX2)+(dY1-dY2)*(dY1-dY2);
          if (dDist < dR*dR) {
            return *it;
          }
        }
      }
      return boost::shared_ptr<CJPDAFilter>();
    }
  }
  return boost::shared_ptr<CJPDAFilter>();
}

void CJPDATracker::InitializeNewObjects() {

  try {
    int n=0;
    string sErrorMsg;

    for (auto it = _vExtractedClusters.begin(); it != _vExtractedClusters.end(); ++it) {
    /*
    for (auto it=_vClusterGroup.begin(); it!=_vClusterGroup.end(); ++it, ++n) {
      vector<boost::shared_ptr<SPointCluster>> vClusters;
      for (auto it2=it->begin(); it2 != it->end(); ++it2) {
        vClusters.push_back(_vExtractedClusters[*it2]);
      }
      */
      vector<boost::shared_ptr<SPointCluster>> vClusters;
      vClusters.push_back(*it);

      boost::shared_ptr<SClassificationResult> pClassificationResult = _pClassifier->Proc(vClusters);
      for (auto itPerson = pClassificationResult->vPersons.begin(); itPerson != pClassificationResult->vPersons.end(); ++itPerson) {
        boost::shared_ptr<CJPDAEllipseFilter> pFilter(new CJPDAEllipseFilter(_nParticleNum));
        pFilter->ResetParticles(_CurrentFilterData, itPerson->vIDs);
        if (_bUseJIPDA) {
          pFilter->SetExistanceRate(itPerson->dProb);
        }
        if (CheckCollision(pFilter, sErrorMsg)) {
          WriteEventLog("Init Failed (Person): CheckCollision Failed", sErrorMsg);
        }
        else {
          ostringstream oss1; oss1 << "Init OK Person #" << pFilter->GetPFID();
          ostringstream oss; oss << "IDs: [" << itPerson->vIDs.front() << ":" << itPerson->vIDs.back() << "]";
          WriteEventLog(oss1.str() , oss.str());
          if (_CurrentConfig._bSetPersonSplittable) {
            pFilter->SetSplittable(true);
          }
          pFilter->SetSizeValues(_CurrentConfig._dPersonR1Ave, _CurrentConfig._dPersonR1StdDev, 
            _CurrentConfig._dPersonR1Max, _CurrentConfig._dPersonR1Min, 
            _CurrentConfig._dPersonR2Ave, _CurrentConfig._dPersonR2StdDev, 
            _CurrentConfig._dPersonR2Max, _CurrentConfig._dPersonR2Min);
          pFilter->SetNoiseValues(_CurrentConfig._dPersonXN, _CurrentConfig._dPersonRadN, 
            _CurrentConfig._dPersonVN1, _CurrentConfig._dPersonRVN1, 
            _CurrentConfig._dPersonVN2, _CurrentConfig._dPersonRVN2, 
            _CurrentConfig._dPersonLowVelThr, _CurrentConfig._dPersonRVNLowVel);
          pFilter->SetLikelihoodFunctionParams(_CurrentConfig._dPersonSigma, 
            _CurrentConfig._dPersonOtherMOLenAve, _CurrentConfig._dPersonOtherMOLenStdDev, 
            _CurrentConfig._dPersonFPExistLen, _CurrentConfig._dPersonFPNearProb, 
            _CurrentConfig._dPersonPassProbTotal);
          _vpFilters.push_back(pFilter);
        }
      }
      for (auto itCar = pClassificationResult->vCars.begin(); itCar != pClassificationResult->vCars.end(); ++itCar) {

        auto pFilter2 = new CJPDARectangleFilter(_nParticleNum);
//        pFilter2->ResetParticles(_CurrentFilterData, vIDs, Info);
//        pFilter2->ResetParticles(*_CurrentFilterData._pLaserData->GetCo(), itCar->vInitVec);
        pFilter2->SetSplittable(true);
        boost::shared_ptr<CJPDAFilter> pFilter(pFilter2);

        if (CheckCollision(pFilter, sErrorMsg)) {
          WriteEventLog("Init Failed (Car): CheckCollision Failed" , sErrorMsg);
        }
        else {
          ostringstream oss1; oss1 << "Init OK Car #" << pFilter->GetPFID();
          ostringstream oss; oss << "IDs: [" << itCar->vIDs.front() << ":" << itCar->vIDs.back() << "]";          WriteEventLog(oss1.str() , oss.str());
          _vpFilters.push_back(pFilter);
        }

      }
    }
  }
  catch (std::exception &e) {
    cout << "initialize error: " << e.what() << endl;
  }

}

void CJPDATracker::PredictPF() {

  if (_bPredictMT) {
    for (auto itP = _vpFilters.begin(); itP != _vpFilters.end(); ++itP) {
      (*itP)->SetTimeStep(_dStepTime);
    }
    vector<boost::shared_ptr<boost::thread> > vpThreads;
    for (auto itP = _vpFilters.begin(); itP != _vpFilters.end(); ++itP) {
      boost::shared_ptr<boost::thread> pThread(
        new boost::thread(
        boost::bind(&CJPDAFilter::Predict, 
        boost::ref(**itP), 
        boost::ref(_CurrentFilterData))));
      vpThreads.push_back(pThread);
    }
    for (size_t i=0; i<vpThreads.size(); ++i) {
      vpThreads[i]->join();
    }
  }
  else {
    for (auto itP = _vpFilters.begin(); itP != _vpFilters.end(); ++itP) {
      (*itP)->SetTimeStep(_dStepTime);
      (*itP)->Predict(_CurrentFilterData);
    }
  }

  if (_bUseJIPDA) {
    //存在確率アップデート
    double _dDisappearRate = 0.05;
    for (auto itP = _vpFilters.begin(); itP != _vpFilters.end(); ++itP) {
      double dE = (*itP)->GetExistanceRate();
      double dNE = (1.0-dE)+dE*_dDisappearRate;
      dE *= (1.0-_dDisappearRate);

      double dENew = dE/(dE+dNE);
      (*itP)->SetExistanceRate(dENew);

      if (_CurrentConfig._bShowDebugString) {
        cout << "Filter #" << (*itP)->GetPFID() << " ExRate: " << (*itP)->GetExistanceRate() << "->" << dENew << endl;
      }
    }
  }
}

void BuildIntervalBitSet(boost::dynamic_bitset<>& rResult, const boost::shared_ptr<CJPDAFilter> &pFilter1, int nInterval) {

  const auto &rOriginal = pFilter1->GetCorreBits();
  size_t nResSize = rOriginal.size()/nInterval;
  rResult.resize(nResSize);
  for (size_t n=0; n<nResSize; ++n) {
    rResult[n] = rOriginal[n*nInterval];
  }
}


bool HasCorrelation(const boost::shared_ptr<CJPDAFilter> &pFilter1, 
                    const boost::shared_ptr<CJPDAFilter> &pFilter2, int nInterval=1) 
{
  if (nInterval == 1) {
    return (pFilter1->GetCorreBits() & pFilter2->GetCorreBits()).any();
  }
  else {
    boost::dynamic_bitset<> vRes1;
    BuildIntervalBitSet(vRes1, pFilter1, nInterval);
    boost::dynamic_bitset<> vRes2;
    BuildIntervalBitSet(vRes2, pFilter2, nInterval);
    return (vRes1 & vRes2).any();
  }
}

bool HasCorrelation2(const boost::shared_ptr<CJPDAFilter> &pFilter1, 
                    const boost::shared_ptr<CJPDAFilter> &pFilter2) 
{
  return overlap(pFilter1->GetMatchedRangeID(), pFilter2->GetMatchedRangeID());
}

void CJPDATracker::SPFCluster::ConstructGroup(
                          const boost::dynamic_bitset<> &rvCorr,
                          const std::vector<boost::shared_ptr<CJPDAFilter> > &rvpFilters,
                          int nLRFPointNum
                          )
{
  _nLRFPointNum = nLRFPointNum;

  std::vector<size_t> _vnPFNo;
  for (size_t j=0; j<rvCorr.size(); ++j) {
    if (rvCorr[j]) _vnPFNo.push_back(j);
  }
  _bStrideBorder = false;
  for (auto it=_vnPFNo.begin(); it!=_vnPFNo.end(); ++it) {
    //一周またぎの場合の判定
    if (rvpFilters[*it]->GetMatchedRangeID().lower() < 0) _bStrideBorder = true;
    _vFilterSorted.push_back(rvpFilters[*it]);
  }
  SetInterval(_nInterval);
}

bool IsFirstAngleSmall(const std::pair<size_t, CJPDAFilter::RangeInterval> &p1, const std::pair<size_t, CJPDAFilter::RangeInterval> &p2) {
  return (p1.second.lower() < p2.second.lower());
}



void CJPDATracker::SPFCluster::SetInterval(int n) {
  _nInterval = n;

  _GroupRange.set_empty();
  _GroupRangeOrg.set_empty();
  _vCorrelation.resize(_nLRFPointNum);
  _vCoverRangeSorted.clear();


  for (auto it=_vFilterSorted.begin(); it!=_vFilterSorted.end(); ++it) {
    auto RangeOrg = (*it)->GetMatchedRangeID();
    int nLower = (int)(ceil(RangeOrg.lower()/(double)(_nInterval))*_nInterval);
    int nUpper = RangeOrg.upper()/_nInterval*_nInterval;    

    if (_bStrideBorder && (nUpper > _nLRFPointNum/2)) {
      nLower -= _nLRFPointNum;
      nUpper -= _nLRFPointNum;
    }
    _vCoverRangeSorted.push_back(CJPDAFilter::RangeInterval(nLower, nUpper));
    _GroupRange = hull(_vCoverRangeSorted.back(), _GroupRange);
    _vCorrelation |= (*it)->GetCorreBits();

    int nLower2 = RangeOrg.lower();
    int nUpper2 = RangeOrg.upper();
    if (_bStrideBorder && (nUpper2 > _nLRFPointNum/2)) {
      nLower2 -= _nLRFPointNum;
      nUpper2 -= _nLRFPointNum;
    }
    _GroupRangeOrg = hull(CJPDAFilter::RangeInterval(nLower2, nUpper2), _GroupRangeOrg);
  }
}

void CJPDATracker::SPFCluster::BuildOrders() {

  for (size_t i=0; i<_vFilterSorted.size(); ++i) {
    _vFilterSorted[i]->SetInterval(_nInterval);
    _vFilterSorted[i]->SetPointCorreBitsGroup(_vCorrelation);
  }

  vector<pair<size_t, CJPDAFilter::RangeInterval> > vRangeSorted2;
  for (size_t i=0; i<_vCoverRangeSorted.size(); ++i) {
    vRangeSorted2.push_back(make_pair(i, _vCoverRangeSorted[i] ));
  }
  std::sort(vRangeSorted2.begin(), vRangeSorted2.end(), &IsFirstAngleSmall);

  _v2nOrders.clear();
  _vCoverRangeSorted.clear();
  auto vFilterBuf = _vFilterSorted;
  _vFilterSorted.clear();
  for (size_t i=0; i<vRangeSorted2.size(); ++i) {
    _vFilterSorted.push_back(vFilterBuf[vRangeSorted2[i].first]);
    _vCoverRangeSorted.push_back(vRangeSorted2[i].second);
  }

  if (s_bShowMsg) {
    cout << "Group Interval: " << _nInterval << endl;
    for (size_t i=0; i<_vCoverRangeSorted.size(); ++i) {
      cout << "PF#" << _vFilterSorted[i]->GetPFID() ;
      cout << " (" << _vFilterSorted[i]->GetMatchedRangeID().lower() << ":" << _vFilterSorted[i]->GetMatchedRangeID().upper() << ")->(" << _vCoverRangeSorted[i].lower() << ":" << _vCoverRangeSorted[i].upper() << ") ";
    }
    cout << endl;
  }
  //v2nOrdersにありうるPFの順序を列挙
  size_t j=0;
  _v2nOrders.clear();

  if (boost::numeric::empty(_GroupRange)) {
//    cout << "Empty GroupRange " << endl;
    if (_vFilterSorted.size() != 1) {
      throw std::logic_error("something wrong: Group empty but includes several filters");//無いはず
    }
    _v2nOrders.push_back(vector<size_t>());
    return;
  }
  if (s_bUseJIPDA) {
    //JIPDA用の並べ方
    for (size_t j=0; j<_vFilterSorted.size(); ++j) {
      vector<size_t> vnOrder; vnOrder.push_back(j);
      vector<size_t> vnNotAdded;
      for (size_t i=0; i<_vFilterSorted.size(); ++i) {
        if (i!=j) vnNotAdded.push_back(i);
      }
      ConstructOrdersNew(vnOrder, vnNotAdded);
    }
    //空のOrderを追加
    _v2nOrders.push_back(vector<size_t>());
  }
  else {
    //最初のPFになりうるものを列挙
    int nFirstAngle = _vCoverRangeSorted.front().lower();
    vector<size_t> vFirstFilters;
    for (size_t i=0; i<_vCoverRangeSorted.size(); ++i) {
      if (_vCoverRangeSorted[i].lower() == nFirstAngle) {
        vFirstFilters.push_back(i);
      }
    }
    if (s_bShowMsg) {
      cout << "first filter: ";
      for (auto it =vFirstFilters.begin(); it!=vFirstFilters.end(); ++it) {
        cout << (*it) << " ";
      }
      cout << endl;
    }
    for (auto it =vFirstFilters.begin(); it!=vFirstFilters.end(); ++it, ++j) {
      vector<size_t> vnOrder; vnOrder.push_back(*it);
      vector<size_t> vnNotAdded;
      for (size_t i=0; i<_vFilterSorted.size(); ++i) {
        if (i!=j) vnNotAdded.push_back(i);
      }
      ConstructOrders(vnOrder, vnNotAdded);
    }
  }

  if (s_bShowMsg) {
    cout << "Before Order" << endl;
    cout << "PFNum=" << _vFilterSorted.size() << " OrderNum=" << _v2nOrders.size() << endl;
    for (size_t i=0; i<_v2nOrders.size(); ++i) {
      const auto &rOrder = _v2nOrders[i];
      cout << " Order#" << i << " Num=" << rOrder.size();
      for (auto it=rOrder.begin(); it!=rOrder.end(); ++it) {
        cout << " PF#" << _vFilterSorted[*it]->GetPFID() << "[" << _vCoverRangeSorted[*it].lower() << ":" << _vCoverRangeSorted[*it].upper() << "] ";
      }
      cout << endl;
    }
    cout << endl;
  }

  auto vOrderBuf = _v2nOrders;
  _v2nOrders.clear();
  for (size_t i=0; i<vOrderBuf.size(); ++i) {
    const auto &rOrder = vOrderBuf[i];
    if (IsOrderOK(rOrder)) _v2nOrders.push_back(rOrder);
  }


  if (s_bShowMsg) {
    cout << "After Order" << endl;
    cout << "PFNum=" << _vFilterSorted.size() << " OrderNum=" << _v2nOrders.size() << endl;
    for (size_t i=0; i<_v2nOrders.size(); ++i) {
      const auto &rOrder = _v2nOrders[i];
      cout << " Order#" << i << " Num=" << rOrder.size();
      for (auto it=rOrder.begin(); it!=rOrder.end(); ++it) {
        cout << " PF#" << _vFilterSorted[*it]->GetPFID() << "[" << _vCoverRangeSorted[*it].lower() << ":" << _vCoverRangeSorted[*it].upper() << "] ";
      }
      cout << endl;
    }
    cout << endl;
  }

  if (_v2nOrders.empty()) {
    ostringstream ss;
    ss << __FUNCTION__ << " Something wrong: Orders empty";
    throw std::logic_error(ss.str().c_str());
  }
}

bool CJPDATracker::SPFCluster::IsOrderOK(const std::vector<size_t> &rOrder) {

  //Orderの整合性チェック
  //例えばCoverRangeが#1:[10:20] #2:[21:21] #3:[15:21]となっていた場合，仮説の生成はできない．
  //PF#2の受け持ち範囲は[21:21]しかありえないので，その後ろにPF#3を持ってくることはできないため．
  //#1:[10:21] #2:[10:24] #2:[10:11] とかもダメ．#1,#2の最小は[10:10],[11:11]なので，#3は最低でも[12:x]の範囲にする必要がある．
  //ついでに同じ順序を2回登録してしまった場合に片方を消す
  if (rOrder.size() >= 2) {   
    int nMin = _vCoverRangeSorted[rOrder.front()].lower();
    int nMax = _vCoverRangeSorted[rOrder.back()].upper();
    for (int j=0; j<(int)rOrder.size(); ++j) {
      int nUpperThr = nMin + _nInterval*(j-1);
      if (_vCoverRangeSorted[rOrder[j]].upper() <= nUpperThr) {
        return false;
      }
      int nLowerThr = nMax - _nInterval*((int)rOrder.size()-j-2);
      if (_vCoverRangeSorted[rOrder[j]].lower() >= nLowerThr) {
        return false;
      }
    }
  }

  //同じ順序の仮説が多重登録されないようにする
  for (size_t j=0; j<_v2nOrders.size(); ++j) {
    const auto &rTargetOrder = _v2nOrders[j];
    if (rOrder.size() != rTargetOrder.size()) continue;
    bool bOK = false;
    for (size_t k=0; k<rOrder.size(); ++k) {
      if (rOrder[k] != rTargetOrder[k]) {
        bOK = true;
        break;
      }
    }
    if (!bOK) {
      return false;
    }
  }
  return true;
}

//新版
string RelationToStringNew(int nRelation) {
  if      (nRelation == 0) return "Same";                      // 0:全点共有
  else if (nRelation == 1) return "Contain-or-InverseOverlap"; // 1:内包 or 一部共有(逆向き)
  else if (nRelation == 2) return "Contain-or-Overlap";        // 2:内包 or 一部共有
  else if (nRelation == 3) return "Contain";                   // 3:内包
  else if (nRelation == 4) return "Overlap";                   // 4:一部を共有
  else if (nRelation == 5) return "NoOverlap";                 // 5:無関係
  else return "Unknown";
}

bool IsRelationLeft(const CJPDAFilter::RangeInterval &Range1, const CJPDAFilter::RangeInterval &Range2) {
  if (Range1.lower() < Range2.lower()) return true;
  else if ((Range1.lower() == Range2.lower()) && (Range2.upper() <= Range1.upper())) {
    return true;
  }
  return false;
}

int GetPFRelationNew(const CJPDAFilter::RangeInterval &Range1, const CJPDAFilter::RangeInterval &Range2) {
  if ((Range1.upper() <  Range2.lower())) return 5; //重なりなし
  if ((Range1.lower() == Range2.lower()) && (Range1.upper() == Range2.upper())) return 0;
  if ((Range1.lower() == Range2.lower()) && (Range2.upper() <  Range1.upper())) return 1;
  if ((Range1.lower() <  Range2.lower()) && (Range1.upper() == Range2.upper())) return 2;
  if ((Range1.lower() <  Range2.lower()) && (Range2.upper() <  Range1.upper())) return 3;
  if ((Range1.lower() <  Range2.lower()) && (Range1.upper() <  Range2.upper())) return 4;
  return 6; //無いはず
}

void CJPDATracker::SPFCluster::ConstructOrdersNew(vector<size_t> vnCurrent, vector<size_t> vnNotAdded) 
{

  if (vnCurrent.empty()) {
    ostringstream oss; oss << "something wrong: vnCurrent empty";
    throw std::logic_error(oss.str().c_str());
  }

  _v2nOrders.push_back(vnCurrent); //結果登録

  size_t nLastID = vnCurrent.back();
  const boost::shared_ptr<CJPDAFilter> &pFilter1 = _vFilterSorted[nLastID];
  const auto &rRange1 = _vCoverRangeSorted[nLastID];
  vector<size_t> vnLefts;
  for (auto it=vnNotAdded.begin(); it!=vnNotAdded.end(); ++it) {
    if (IsRelationLeft(rRange1, _vCoverRangeSorted[*it])) vnLefts.push_back(*it);
  }

  if (s_bShowMsg) {
    ccout->SetColor(ColorCout::eGreen);
    ccout << "Add: ";
    for (size_t i=0; i<vnCurrent.size(); ++i) {
      ccout << vnCurrent[i] << " ";
    }
    ccout << endl;
    ccout << " LeftNum=" << vnLefts.size() << " NotAddedNum=" << vnNotAdded.size() << endl;
    ccout << " Left: " ;
    for (auto it=vnLefts.begin(); it!=vnLefts.end(); ++it) {
      ccout << "[" << _vCoverRangeSorted[*it].lower() << ":" << _vCoverRangeSorted[*it].upper() << "], ";
    }
    ccout << endl;
  }

  if (vnLefts.empty()) return; //終了判定

  for (auto it=vnLefts.begin(); it!=vnLefts.end(); ++it) {
    const boost::shared_ptr<CJPDAFilter> &pFilter2 = _vFilterSorted[*it];
    const auto &rRange2 = _vCoverRangeSorted[*it];
    int nRelation = GetPFRelationNew(rRange1, rRange2);
    if (s_bShowMsg) {
      ccout->SetColor(ColorCout::eGreen);
      ccout <<"Relation #" << pFilter1->GetPFID() << ":#" << pFilter2->GetPFID() << "=";
      ccout << RelationToStringNew(nRelation);
    }

    vector<size_t> vnNotAddedNext = vnNotAdded;
    vnNotAddedNext.erase(std::find(vnNotAddedNext.begin(), vnNotAddedNext.end(), *it));

    //[1-2-1]の配列で1の方が手前というのはありえない
    double dDist1 =  pFilter1->GetAverageDistFromLRF();
    double dDist2 =  pFilter2->GetAverageDistFromLRF();
    
    if ((nRelation == 0) || (nRelation == 2)) {
      vector<size_t> vnNext1 = vnCurrent; vnNext1.push_back(*it);
      ConstructOrdersNew(vnCurrent, vnNotAddedNext); //[1]
      ConstructOrdersNew(vnNext1, vnNotAddedNext);   //[1-2]
      if (pFilter1->IsSplittable() && (dDist1 > dDist2)) { 
        vector<size_t> vnNext2 = vnCurrent; vnNext2.push_back(*it); vnNext2.push_back(nLastID);
        ConstructOrdersNew(vnNext2, vnNotAddedNext);   //[1-2-1]
      }
    }
    else if (nRelation == 1) {
      vector<size_t> vnNext1 = vnCurrent;
      vnNext1.pop_back(); vnNext1.push_back(*it); vnNext1.push_back(nLastID);
      ConstructOrdersNew(vnCurrent, vnNotAddedNext); //[1]
      ConstructOrdersNew(vnNext1, vnNotAddedNext);   //[2-1]
      if (pFilter1->IsSplittable() && (dDist1 > dDist2)) { 
        vector<size_t> vnNext2 = vnCurrent; vnNext2.push_back(*it); vnNext2.push_back(nLastID);
        ConstructOrdersNew(vnNext2, vnNotAddedNext);   //[1-2-1]
      }
    }
    else if (nRelation == 3) {
      ConstructOrdersNew(vnCurrent, vnNotAddedNext); //[1]
      if (pFilter1->IsSplittable() && (dDist1 > dDist2)) { 
        vector<size_t> vnNext2 = vnCurrent; vnNext2.push_back(*it); vnNext2.push_back(nLastID);
        ConstructOrdersNew(vnNext2, vnNotAddedNext);   //[1-2-1]
      }
    }
    else if ((nRelation == 4) || (nRelation == 5)) {
      vector<size_t> vnNext1 = vnCurrent; vnNext1.push_back(*it);
      ConstructOrdersNew(vnNext1, vnNotAddedNext); //[1-2]
    }
    else {
      ccout->SetColor(ColorCout::eRed);
      ccout <<"Relation=" << nRelation << " Something wrong" << endl;
    }
  }
}



//旧版
template <typename T>
bool subset2(const T& r1, const T& r2) {
  return (subset(r1, r2) && (r1.lower() != r2.lower()) && (r1.upper() != r2.upper()));
}
string RelationToString(int nRelation) {
  if      (nRelation == 0) return "Same";                 // 0:全点共有
  else if (nRelation == 1) return "First-Second";         // 1:pFilter1 -> pFilter2
  else if (nRelation == 2) return "Second-First";         // 2:pFilter2 -> pFilter1
  else if (nRelation == 3) return "Second-Contain-First"; // 3:pFilter1 が pFilter2に含まれる
  else if (nRelation == 4) return "First-Contain-Second"; // 4:pFilter2 が pFilter1に含まれる
  else if (nRelation == 5) return "NoOverlap";            // 5:無関係
  else return "Unknown";
}
int GetPFRelation(const CJPDAFilter::RangeInterval &Range1, const CJPDAFilter::RangeInterval &Range2) {

  if (!overlap(Range1, Range2)) return 5;
  else if (equal(Range1, Range2) ) return 0;
  else if (subset2(Range1, Range2)) return 3;
  else if (subset2(Range2, Range1)) return 4;
  else {
    if (Range1.lower() > Range2.lower()) return 2;
    else if ((Range1.lower() == Range2.lower()) && (Range1.upper() > Range2.upper())) return 2;
    else return 1;
  }
}

void CJPDATracker::SPFCluster::ConstructOrders(vector<size_t> vnCurrent, vector<size_t> vnNotAdded) 
{

  if (vnCurrent.empty()) {
    ostringstream oss; oss << "something wrong: vnCurrent empty";
    throw std::logic_error(oss.str().c_str());
  }
  size_t nLastID = vnCurrent.back();
  const boost::shared_ptr<CJPDAFilter> &pFilter1 = _vFilterSorted[nLastID];
  const auto &rRange1 = _vCoverRangeSorted[nLastID];
    
  //終了判定
  if (rRange1.upper() == _GroupRange.upper()) {
    if (s_bShowMsg) {
      ccout->SetColor(ColorCout::eGreen);
      ccout << "Add1: ";
      for (size_t i=0; i<vnCurrent.size(); ++i) {
        ccout << vnCurrent[i] << " ";
      }
      ccout << endl;
    }
    _v2nOrders.push_back(vnCurrent);
    if (vnNotAdded.empty()) return;
  }
  else if (vnNotAdded.empty()) {
    //残りがFPの場合
    if (s_bShowMsg) {
      ccout->SetColor(ColorCout::eYellow);
      ccout << "Add2: ";
      for (size_t i=0; i<vnCurrent.size(); ++i) {
        ccout << vnCurrent[i] << " ";
      }
      ccout << endl;
    }
    _v2nOrders.push_back(vnCurrent);
    return;
  }

  for (auto it=vnNotAdded.begin(); it!=vnNotAdded.end(); ++it) {
    const boost::shared_ptr<CJPDAFilter> &pFilter2 = _vFilterSorted[*it];
    const auto &rRange2 = _vCoverRangeSorted[*it];
    int nRelation = GetPFRelation(rRange1, rRange2);
    if (s_bShowMsg) {
      switch (nRelation) {
      case  0: ccout->SetColor(ColorCout::eCyan);    break; //全点共有
      case  1: ccout->SetColor(ColorCout::eGreen);   break; //pFilter1 -> pFilter2
      case  2: ccout->SetColor(ColorCout::eMagenta); break; //pFilter2 -> pFilter1
      case  3: ccout->SetColor(ColorCout::eYellow);  break; //pFilter1 が pFilter2に含まれる
      case  4: ccout->SetColor(ColorCout::eYellow);  break; //pFilter2 が pFilter1に含まれる
      default: ccout->SetColor(ColorCout::eRed);     break; //無関係       
      }
      ccout <<"Relation #" << pFilter1->GetPFID() << ":#" << pFilter2->GetPFID() << "=";
      ccout << RelationToString(nRelation);
    }
    if ((nRelation == 0) || (nRelation == 1) || (nRelation == 3)) {
      if (s_bShowMsg) ccout << " ->Type" << nRelation << endl;
      vector<size_t> vnCurrent2 = vnCurrent;
      vnCurrent2.push_back(*it);
      vector<size_t> vnNotAdded2 = vnNotAdded;
      vnNotAdded2.erase(vnNotAdded2.begin()+(it-vnNotAdded.begin())); //vnNotAddedからitだけ消したものをコピー作成
      ConstructOrders(vnCurrent2, vnNotAdded2);
    }
    else if (nRelation == 4) { //人同士->普通は起こらない 車と人->人が車体を遮る
//      _vType4Pairs.push_back(make_pair(nLastID, *it));
      //todo 中心の平均距離しかみてない．異種パーティクルの場合はダメかも？
      double dDist1 =  pFilter1->GetAverageDistFromLRF();
      double dDist2 =  pFilter2->GetAverageDistFromLRF();
      if (s_bShowMsg) {
        ccout << " ->Type" << nRelation << " (notice!!) " <<  endl;
        ccout << " dist: " << dDist1 << " " << dDist2 << ":" ;
      }
      if (dDist1 > dDist2) { 
        //奥側が大きいのでありうる
        //パーティクルの大きさを見るべきだが，重なったのは削除されるのでまあいいか．       
        //TODO //0-1-0, 0-1 しか作られない．1-0もいる？
        if (s_bShowMsg) ccout << " order OK";
        vector<size_t> vnCurrent2 = vnCurrent;
        vnCurrent2.push_back(*it);
        vector<size_t> vnNotAdded2 = vnNotAdded;
        vnNotAdded2.erase(vnNotAdded2.begin()+(it-vnNotAdded.begin())); //vnNotAddedからitだけ消したものをコピー作成
        ConstructOrders(vnCurrent2, vnNotAdded2);
        vector<size_t> vnNotAdded3 = vnNotAdded2;
        vnNotAdded3.push_back(nLastID); //一個前のが2階出てくる
        ConstructOrders(vnCurrent2, vnNotAdded3);
      }
      else {
        //奥側が小さかったら明らかにおかしい．
        if (s_bShowMsg) {
          ccout << " Order not correct!!" << endl;
        }
      }
    }
    else {
      if (s_bShowMsg) {
        ccout << " ->Type" << nRelation << " (do nothing)" << endl;
      }
    }
  }
}

size_t CJPDATracker::SPFCluster::GetTotalValidLRFPointsNum(const SJPDAFilterData& rCurrentFilterData) const {
  const auto &rvStatus = rCurrentFilterData._vPointStatus;

  size_t nTotal = 0;
  for (int n=_GroupRange.lower(); n<=_GroupRange.upper(); n+=_nInterval) {
    int n2 = n;
    if (n<0) n2 += _nLRFPointNum;
    if (rvStatus[n2] == 1) ++nTotal;
  }
  return nTotal;
}

//PF同士の相関を調べる
void MakeCorrelation(const std::vector<boost::shared_ptr<CJPDAFilter> > &rvFilter, vector<boost::dynamic_bitset<> > &rResult, int nInterval)
{
  rResult.clear();
  if (rvFilter.empty()) return;
  else if (rvFilter.size() == 1) {
    boost::dynamic_bitset<> v(1); 
    v[0] = true;
    rResult.push_back(v);
    return;
  }
  vector<boost::dynamic_bitset<> > vCorrelations(rvFilter.size());
  for (size_t i=0; i<rvFilter.size(); ++i) {
    auto &rCorrelation = vCorrelations[i];
    rCorrelation.resize(rvFilter.size());
    for (size_t j=0; j<rvFilter.size(); ++j) {
      if ((i==j) || HasCorrelation(rvFilter[i], rvFilter[j], nInterval)) {
        rCorrelation[j] = 1;
      }
      else  rCorrelation[j] = 0;
    }
  }
  vector<int> _vnGroupNo;
  while(true) {
    int nMergedNum = 0;
    for (size_t i=0; i<vCorrelations.size(); ++i) {
      for (size_t j=i; j<vCorrelations.size(); ++j) {
        if ((vCorrelations[i] != vCorrelations[j]) &&
            (vCorrelations[i] & vCorrelations[j]).any()) 
        {
          ++nMergedNum;
          vCorrelations[i] |= vCorrelations[j];
          vCorrelations[j] = vCorrelations[i];
        }
      }
    }
    if (nMergedNum == 0) break;
  }
  //重複を除去
  for (size_t i=0; i<vCorrelations.size(); ++i) {
    if (std::find(rResult.begin(), rResult.end(), vCorrelations[i]) == rResult.end()) {
      rResult.push_back(vCorrelations[i]);
    }
  }
}

void CJPDATracker::MakeGroup() {

  vector<boost::dynamic_bitset<> > vCorrelations;
  MakeCorrelation(_vpFilters, vCorrelations, 1);
  _vGroupFirst.clear();
  //_vGroupFirstに相関のあるパーティクルセットを格納
  for (size_t i=0; i<vCorrelations.size(); ++i) {
    const auto &rv1 = vCorrelations[i];
    boost::shared_ptr<SPFCluster> pGroup(new SPFCluster(rv1, _vpFilters, _nLRFPointNum));
    _vGroupFirst.push_back(pGroup);
  }
  
  //間引きすると_vGroupFirstが分離されることがある．_vGroupは間引き後のデータを格納
  _vGroup.clear();
  if (_dIntervalStandardLen > 0) {
    //間引きの設定
    for (size_t k=0; k<_vGroupFirst.size(); ++k) {
      const auto &vFilterSorted = _vGroupFirst[k]->GetFilters();
      //Intervalを計算
      double dAverageDist = 0;
      for (size_t j=0; j<vFilterSorted.size(); ++j) {
        dAverageDist += (vFilterSorted[j]->GetAverageDistFromLRF() / vFilterSorted.size());
      }
      int nInterval = max(1, (int)round(_dIntervalStandardLen/dAverageDist));
      _vGroupFirst[k]->SetInterval(nInterval);
      if (nInterval == 1) {
        _vGroup.push_back(_vGroupFirst[k]);
      }
      else {
        vector<boost::dynamic_bitset<> > vCorrelations2;
        MakeCorrelation(vFilterSorted, vCorrelations2, nInterval);
        for (size_t i=0; i<vCorrelations2.size(); ++i) {
          const auto &rv1 = vCorrelations2[i];
          if (rv1.any() && (!_vGroupFirst[k]->GetFilters().empty())) {
            boost::shared_ptr<SPFCluster> pGroup(new SPFCluster(rv1, *_vGroupFirst[k]));
            _vGroup.push_back(pGroup);
          }
        }
      }
      if (_CurrentConfig._bShowDebugString) {
        cout << "Group #" << k << " Interval=" << nInterval << " AverageDist=" << dAverageDist << " StandardLen=" << _dIntervalStandardLen << endl;
      }
    }
  }
  else {
    _vGroup =  _vGroupFirst;
  }


  for (auto it=_vGroup.begin(); it!=_vGroup.end(); ++it) {
    if (_CurrentConfig._bShowDebugString) {
      ccout->SetColor(ColorCout::eMagenta);
      cout << endl;
      ccout << "Start BuildOrder #" << it-_vGroup.begin() << endl;
    }
    (*it)->BuildOrders();
  }


  if (_CurrentConfig._bShowDebugString) {
    ccout->SetColor(ColorCout::eGreen);
    ccout << __FUNCTION__ << " Frame: " << _nCurrentFrame << endl;
    for (size_t i=0; i<_vGroup.size(); ++i) {
      ccout << "Group#" << i << endl;
      for (size_t j=0; j<_vGroup[i]->GetFilters().size(); ++j) {
        const auto &rpFilter = _vGroup[i]->GetFilters()[j];
        ccout << " PF#" << rpFilter->GetPFID() << " ";
        ccout << " R2:[" << rpFilter->GetMatchedRangeID().lower() << ":" << 
          rpFilter->GetMatchedRangeID().upper() << "]";
        ccout << endl;
      }
      ccout << endl;
    }
  }
}

void MakeHypoVectors(int nHypoID, const vector<int>& rvnHypoNums, vector<int> &rvnIDVectors) {

  rvnIDVectors.resize(rvnHypoNums.size());
//  cout << "ID=" << nHypoID;
  for(size_t i=0; i<rvnHypoNums.size(); ++i) {
    
    rvnIDVectors[i] = nHypoID%rvnHypoNums[i];
    nHypoID /= rvnHypoNums[i];
//    cout << " G" << i << ":" << rvnIDVectors[i];
  }
//  cout << endl;
}

//(x1,y1)の方が角度が浅ければtrue
bool GetAngleOrder(const boost::shared_ptr<const CCascadedCoords> &pLRFCo, double x1, double y1, double x2, double y2) {

  BoostVec v1(3); v1(0) = x1; v1(1) = y1; v1(2) = 0;
  BoostVec v2(3); v2(0) = x2; v2(1) = y2; v2(2) = 0;

  pLRFCo->VectorGlobalToLocal(v1);
  pLRFCo->VectorGlobalToLocal(v2);

  CAngle r1 = FastMath::table_atan2(v1(1), v1(0));
  CAngle r2 = FastMath::table_atan2(v2(1), v2(0));

  return (r1 < r2);
}



void CJPDATracker::CBetaCalculateHelper::SetTree(CJPDATracker::CBetaCalculateHelper* pPrev, CJPDATracker::CBetaCalculateHelper* pNext) 
{
  mmtimer mt;
  vector<double> vdTimes;

  _pPrev = pPrev;
  _pNext = pNext;
  _vRanges.clear();
  int nLower = _FilterRange.lower();
  int nUpper = _FilterRange.upper();
  int nGroupLower = _GroupRange.lower();
  int nGroupUpper = _GroupRange.upper();

  if (!_pPrev && !_pNext) {
//    _vRanges.push_back(_FilterRange); 
    _vRanges.push_back(_GroupRange);  //JIPDA
  }
  else {
    /* 
     JIPDA対応で番号が飛んでいる場合, 範囲を広げて仮説を生成
     番号が若い方の範囲を広げる
    */
    int nLMin, nLMax, nUMin, nUMax;
    if (_pPrev) {
      auto &rRange1 = _pPrev->_FilterRange;
      auto &rRange2 = this->_FilterRange;

      if (rRange2.lower()-rRange1.upper() > _nInterval) {
//        cout << "mode 1 [" << rRange1.lower() << ":" << rRange1.upper() << "]-[" << rRange2.lower() << ":" << rRange2.upper() << "]" << endl;
        nLMin = nLMax = rRange2.lower();
      }
      else {
        nLMin = rRange2.lower();
        nLMax = rRange1.upper();
        if (nLMin % _nInterval == 0) nLMin = nLMin;
        else nLMin = (nLMin/_nInterval+1)*_nInterval;
        nLMax = (nLMax/_nInterval+1)*_nInterval;
        if (nLMin<nLower) nLMin = nLower;
        if (nLMin<nGroupLower+_nInterval) nLMin = nGroupLower+_nInterval;
        if (nLMax>nUpper) nLMax = nUpper;
      }
    }
    else {
//      nLMin = nLMax = nLower;
      nLMin = nLMax = nGroupLower; //JIPDA
    }
    if (_pNext) {
      auto &rRange1 = this->_FilterRange;
      auto &rRange2 = _pNext->_FilterRange;

      if (rRange2.lower()-rRange1.upper() > _nInterval) {
//        cout << "mode 0 [" << rRange1.lower() << ":" << rRange1.upper() << "]-[" << rRange2.lower() << ":" << rRange2.upper() << "]" << endl;
        nUMin = nUMax = rRange2.lower()-_nInterval;
      }
      else {
        nUMin = rRange2.lower();
        nUMax = rRange1.upper();
        if (nUMin % _nInterval == 0) nUMin -= _nInterval;
        else nUMin = nUMin/_nInterval*_nInterval;
        nUMax = nUMax/_nInterval*_nInterval;
        if (nUMin<nLower) nUMin = nLower;
        if (nUMax>nUpper) nUMax = nUpper;
        if (nUMax>nGroupUpper-_nInterval) nUMax = nGroupUpper-_nInterval;
      }
    }
    else {
//      nUMin = nUMax = nUpper;   
      nUMin = nUMax = nGroupUpper; //JIPDA
    }

    for (int nL=nLMin; nL<=nLMax; nL+=_nInterval) {
      for (int nU=max(nL, nUMin); nU<=nUMax; nU+=_nInterval) {
        if (nL <= nU) {
          _vRanges.push_back(CJPDAFilter::RangeInterval(nL,nU));
          if (pPrev) pPrev->InsertNext(nL-_nInterval, _vRanges.size()-1);
          if (pNext) pNext->InsertPrev(nU+_nInterval, _vRanges.size()-1);
        }
      }
    }
    if (_vRanges.empty()) {
      ccout->SetColor(ColorCout::eRed);
      ccout << "Error empty range!!" << endl;
      cout << " PFID #" << _pFilter->GetPFID() << endl;
      cout << nLMin << " " << nLMax << endl;
      cout << nUMin << " " << nUMax << endl;

      _vRanges.push_back(CJPDAFilter::RangeInterval::empty());
//      if (pPrev) pPrev->InsertNext(nLMin, _vRanges.size()-1);
//      if (pNext) pNext->InsertPrev(nLMin, _vRanges.size()-1);
    }

    if (s_bShowMsg) {
      cout << "FilterRange: " << _FilterRange.lower() << ":" << _FilterRange.upper();
      cout << " Interval: " << _nInterval << endl;
      if (_pPrev) cout << " Prev:" << _pPrev->_FilterRange.lower() << ":" << _pPrev->_FilterRange.upper() << endl;
      else cout << " [no prev]" << endl;

      if (_pNext) cout << " Next:" << _pNext->_FilterRange.lower() << ":" << _pNext->_FilterRange.upper() << endl;
      else cout << " [no next]" << endl;
      cout << " LMin: " << nLMin << " LMax: " << nLMax << endl;
      cout << " UMin: " << nUMin << " UMax: " << nUMax << endl;
      cout << " Ranges: ";
      for (size_t i=0; i<_vRanges.size(); ++i) {
        cout << "[" << _vRanges[i].lower() << ":" << _vRanges[i].upper() << "], ";
      }
      cout << endl;
    }
  }

  _vBeta = vector<double>(_vRanges.size(), 0);
  _vLeft = vector<double>(_vRanges.size(), -1);
  _vRight = vector<double>(_vRanges.size(), -1);
  _vP.clear();
  _vPBackUpID.clear();
  _pFilter->PrepareMemories(_vRanges.size());
  vdTimes.push_back(mt.elapsed());


  for (size_t i=0; i<_vRanges.size(); ++i) {
    const auto &rRange = _vRanges[i];
    vector<CJPDAFilter::RangeInterval> vRange; vRange.push_back(rRange);
    double dP; int nBufNo;
    boost::tie(dP, nBufNo) = _pFilter->CalcP(vRange);
    _vP.push_back(dP);
    _vPBackUpID.push_back(nBufNo);

    if (dP <= DBL_MIN) {
      ccout->SetColor(ColorCout::eRed);
      ccout << "Error P=0 PFID=" << _pFilter->GetPFID() << " [" << rRange.lower() << ":" << rRange.upper()  << "]" << endl;
    }
  }
  vdTimes.push_back(mt.elapsed());
}

void CJPDATracker::CBetaCalculateHelper::CalcBeta() {

  for (size_t i=0; i<_vRanges.size(); ++i) {
    const auto &rRange = _vRanges[i];
    const auto &rvPrevPos = vPrevs[rRange.lower()];
    const auto &rvNextPos = vNexts[rRange.upper()];

    if (rvPrevPos.empty() && _pPrev) continue; //本来ないはず
    if (rvNextPos.empty() && _pNext) continue; //本来ないはず
    double dTotal = 0;
    if (rvPrevPos.empty() && rvNextPos.empty()) {
      dTotal = 1;
    }
    else if (rvPrevPos.empty()) {
      for (size_t j=0; j<rvNextPos.size(); ++j) {
        dTotal += _pNext->GetRight(rvNextPos[j]);
      }
    }
    else if (rvNextPos.empty()) {
      for (size_t j=0; j<rvPrevPos.size(); ++j) {
        dTotal += _pPrev->GetLeft(rvPrevPos[j]);
      }
    }
    else {
      for (size_t j=0; j<rvPrevPos.size(); ++j) {
        for (size_t k=0; k<rvNextPos.size(); ++k) {
          dTotal += _pNext->GetRight(rvNextPos[k])*_pPrev->GetLeft(rvPrevPos[j]);
        }
      }
    }
    _vBeta[i] = _vP[i]*dTotal;
  }

  if (s_bShowMsg) {
    cout << __FUNCTION__ << " #" << _pFilter->GetPFID() << endl;
    cout << "P value" << endl;
    for (size_t i=0; i<_vP.size(); ++i) {
      cout << _vP[i] << " ";
    }
    cout << endl;
    cout << "R/LValue" << endl;
    for (size_t i=0; i<_vRanges.size(); ++i) {
      const auto &rRange = _vRanges[i];
      const auto &rvPrevPos = vPrevs[rRange.lower()];
      const auto &rvNextPos = vNexts[rRange.upper()];
      cout << " Range#" <<i  << "("<< rRange.lower() << ":" << rRange.upper() << ")" << endl;
      if (rvPrevPos.empty() && _pPrev) {
        cout << "  No Prev Error: " << rvPrevPos.empty() << " " << _pPrev << endl;
      }
      else {
        cout << "  prevNum=" << rvPrevPos.size() << " ";
        for (size_t j=0; j<rvPrevPos.size(); ++j) {
          cout << _pPrev->GetLeft(rvPrevPos[j]) << " ";
        }
        cout << endl;
      }
      if (rvNextPos.empty() && _pNext) {
        cout << "  No Next Error: " << rvNextPos.empty() << " " << _pNext << endl;
      }
      else {
        cout << "  nextNum=" << rvNextPos.size() << " ";
        for (size_t j=0; j<rvNextPos.size(); ++j) {
          cout << _pNext->GetRight(rvNextPos[j]) << " ";
        }
        cout << endl;
      }
    }
    cout << "Beta value" << endl;
    for (size_t i=0; i<_vBeta.size(); ++i) {
      cout << _vBeta[i] << " ";
    }
    cout << endl;
  }

}

double CJPDATracker::CBetaCalculateHelper::GetLeft(size_t n) {

  //計算済み
  if (_vLeft[n]>=0) return _vLeft[n];
  //左端の場合は_vLeft[n]=_vP[n]
  if (!_pPrev) {
    _vLeft[n] = _vP[n];
    return _vLeft[n];
  }
  else {
    //ツリー構造を辿る
    _vLeft[n] = 0;
    const auto &rvPrevs = vPrevs[_vRanges[n].lower()];
    for (auto it=rvPrevs.begin(); it!=rvPrevs.end(); ++it) {
      _vLeft[n] += _pPrev->GetLeft(*it);
    }
    _vLeft[n] *= _vP[n];
    return _vLeft[n];
  }
}

double CJPDATracker::CBetaCalculateHelper::GetRight(size_t n) {

  if (_vRight[n]>=0) return _vRight[n];
  if (!_pNext) {
    _vRight[n] = _vP[n];
    return _vRight[n];
  }
  else {
    _vRight[n] = 0;
    const auto &rvNexts = vNexts[_vRanges[n].upper()];
    for (auto it=rvNexts.begin(); it!=rvNexts.end(); ++it) {
      _vRight[n] += _pNext->GetRight(*it);
    }
    _vRight[n] *= _vP[n];
    return _vRight[n];
  }
}

bool CJPDATracker::SPFCluster::s_bShowMsg = false;
bool CJPDATracker::CBetaCalculateHelper::s_bShowMsg = false;


void CJPDATracker::SolveJPDA() {

  SPFCluster::SetShowMsg(_CurrentConfig._bShowDebugString);
  CBetaCalculateHelper::SetShowMsg(_CurrentConfig._bShowDebugString);

  _vBestHypoDivLines.clear();
  MakeGroup(); //_vGroupに相関のあるパーティクルをまとめる

  //グループごとに仮説生成, 各仮説の中身と尤度を決める
  for (size_t k=0; k<_vGroup.size(); ++k) {
    mmtimer mt1;

    const auto &rAllRange = _vGroup[k]->GetGroupRange();
    const auto  &vFilterSorted = _vGroup[k]->GetFilters();
    const auto  &vCoverRangeSorted = _vGroup[k]->GetCoverRanges();
    int nInterval = _vGroup[k]->GetInterval();
    const auto &v2nOrders = _vGroup[k]->GetOrders();

    //順序行列ごとに仮説生成
    std::vector<std::vector<boost::shared_ptr<CBetaCalculateHelper> >> v2pBetaHelper; //一旦Orderごとにここにまとめる
    std::vector<boost::shared_ptr<SHypothesis> > vpBestHypos; //最尤仮説
    if (_CurrentConfig._bShowDebugString) {
      cout << "Group #" << k << " OrderNum=" << v2nOrders.size() << endl;
    }

    //times
    vector< vector<double> > vTimes(v2nOrders.size());

    for (size_t j=0; j<v2nOrders.size(); ++j) {

      mmtimer mt2;
      const auto &rvOrder = v2nOrders[j];

      if (rvOrder.empty()) {
        size_t nPointsNum = _vGroup[k]->GetTotalValidLRFPointsNum(_CurrentFilterData);
        double dFPFarP = CJPDAFilter::GetFalsePositiveProbDensity();
        double dLikelihood = pow(dFPFarP, (int)nPointsNum);
        boost::shared_ptr<SHypothesis> pHypo(new SHypothesis);
        pHypo->_dLikelihood = dLikelihood;

        vpBestHypos.push_back(pHypo);
        v2pBetaHelper.push_back(std::vector<boost::shared_ptr<CBetaCalculateHelper>>());
                
        if (_CurrentConfig._bShowDebugString) {
          cout << "Order #" << j << " Empty" << endl;
          cout << "PointNUm=" << nPointsNum << " Likelihood: " << dLikelihood << endl;
        }
        vTimes[j].push_back(mt2.elapsed());
      }
      else {
        if (_CurrentConfig._bShowDebugString) {
          cout << "Order #" << j << endl;
          for (size_t i=0; i<rvOrder.size(); ++i) {
            cout << " PF #" << vFilterSorted[rvOrder[i]]->GetPFID() << " ";
          }
          cout << endl;
        }
        std::vector<boost::shared_ptr<CBetaCalculateHelper> > _vpBetaHelper;
        for (size_t i=0; i<rvOrder.size(); ++i) {
          _vpBetaHelper.push_back(boost::shared_ptr<CBetaCalculateHelper>
            (new CBetaCalculateHelper(vFilterSorted[rvOrder[i]], vCoverRangeSorted[rvOrder[i]], nInterval, rAllRange)));
        }
        vTimes[j].push_back(mt2.elapsed()); mt2.restart();
        if (_vpBetaHelper.size() == 1) {
          _vpBetaHelper.front()->SetTree(nullptr, nullptr);
        }
        else {
          //TODO ここで分断を探す ある場合は再構成が必要
          for (size_t i=0; i<_vpBetaHelper.size(); ++i) {
            if (i==0) _vpBetaHelper[i]->SetTree(nullptr, _vpBetaHelper[i+1].get());
            else if (i==rvOrder.size()-1) _vpBetaHelper[i]->SetTree(_vpBetaHelper[i-1].get(), nullptr);
            else _vpBetaHelper[i]->SetTree(_vpBetaHelper[i-1].get(), _vpBetaHelper[i+1].get());
          }
        }
        vTimes[j].push_back(mt2.elapsed()); mt2.restart();

        if (_bUseDynamicProgramming) {
          mmtimer mt;
          //動的計画法を使って解く
          for (size_t i=0; i<_vpBetaHelper.size(); ++i) {
            _vpBetaHelper[i]->CalcBeta();
          }
          double d1 = mt.elapsed();
          v2pBetaHelper.push_back(_vpBetaHelper);
          double d2 = mt.elapsed();
          if (_bCalcBestHypoInDPMode) {
            vpBestHypos.push_back(CalcBestHypo(_vpBetaHelper));
          }
          else vpBestHypos.push_back(boost::shared_ptr<SHypothesis>());
          double d3 = mt.elapsed();

          /*
          if (_vpBetaHelper.size() >= 4) {
            cout << "Frame=" << _nCurrentFrame << " Order=" << j << " size=" << _vpBetaHelper.size() << " Time=" << d1 << "," <<d2 << "," << d3 << endl;
          }
          */
        }
        else {
          mmtimer mt;
          //仮説を列挙して解く
          boost::shared_ptr<SHypothesis> pBestHypo = CalcBetaUsingEnumeration(_vpBetaHelper);
          double d1 = mt.elapsed();
          v2pBetaHelper.push_back(_vpBetaHelper);
          vpBestHypos.push_back(pBestHypo);
          double d2 = mt.elapsed();

          if (_vpBetaHelper.size() >= 4) {
//s           cout << "Frame=" << _nCurrentFrame << " Order=" << j << " size=" << _vpBetaHelper.size() << " Time=" << d1 << "," <<d2  << endl;
          }

        }
        vTimes[j].push_back(mt2.elapsed());
      }
    }

    double dTime1 = mt1.elapsed(); mt1.restart();

    //v2pBeta を変形, Filterごとにまとめる
    vector< vector<pair<double, int>>> vBeta(vFilterSorted.size());
    for (size_t k=0; k<v2nOrders.size(); ++k) {
      const auto &vpBetaHelper = v2pBetaHelper[k];
      const auto &vOrder = v2nOrders[k];
      const auto &pBestHypo = vpBestHypos[k];
      /*
      if (!pBestHypo) {
        ccout->SetColor(ColorCout::eRed);
        ccout << "Order #" << k << " error! no valid hypos" << endl;
        continue;
      }
      */

      double dBetaTotal = 0;
      if (!vpBetaHelper.empty()) {
        for (size_t i=0; i<vpBetaHelper.front()->GetBeta().size(); ++i) {
          dBetaTotal += vpBetaHelper.front()->GetBeta()[i];
        }
        {
          //test
          for (size_t i=1; i<vpBetaHelper.size(); ++i) {
            double dTotal = 0;
            for (size_t k=0; k<vpBetaHelper[i]->GetBeta().size(); ++k) {
              dTotal += vpBetaHelper[i]->GetBeta()[k];
            }
            if (!eps_eq(dTotal/dBetaTotal, 1.0, 0.00001)) {
              if (_CurrentConfig._bShowDebugString) {
                ccout->SetColor(ColorCout::eRed);
                ccout << " Something Wrong!! at Frame" << _nCurrentFrame << endl;
                ccout << dTotal << " " << dBetaTotal << " (at " << i << ")" << endl;
              }
            }
          }
        }
      }
      else {
        if (!pBestHypo) {
          ccout->SetColor(ColorCout::eRed);
          ccout << "Something wrong Order #" << k << " error! no hypo and no BetaHelper" << endl;
          continue;
        }
        dBetaTotal = pBestHypo->_dLikelihood;
      }
      vector< vector<pair<double, int>>> vBeta2(vFilterSorted.size());
      for (size_t i=0; i<vpBetaHelper.size(); ++i) {
        size_t nPos  = vOrder[i];
        for (size_t k=0; k<vpBetaHelper[i]->GetBeta().size(); ++k) {
          double dBeta = vpBetaHelper[i]->GetBeta()[k];
          int nID      = vpBetaHelper[i]->GetBackUpID()[k];
          vBeta2[nPos].push_back(make_pair(dBeta, nID));
        }
      }
      vector<size_t> vEmptyList;
      for (size_t i=0; i<vBeta2.size(); ++i) {
        if (vBeta2[i].empty()) {
          vEmptyList.push_back(i);
        }
      }
      if (_CurrentConfig._bShowDebugString) {
        cout << "Order #" << k <<  " BetaTotal: " << dBetaTotal << endl;
        cout << "Empty PF:";
        for (size_t i=0; i<vEmptyList.size(); ++i) {
          cout << " #" << vFilterSorted[vEmptyList[i]]->GetPFID();
        }
        cout << endl;
      }
      if (_bUseJIPDA) {

        //一部存在しない仮定
        int nLoopNum = (int)pow(2.0, (int)vEmptyList.size());
        for (int n=0; n<nLoopNum; ++n) {
          auto vBeta3 = vBeta2;
          boost::dynamic_bitset<> vExistList(vEmptyList.size());
          int n2 = n;
          for (size_t i=0; i<vEmptyList.size(); ++i) {
            vExistList[i] = (n2%2 == 1);
            n2/=2;
          }
          double dMultiply = 1.0;
          for (size_t k=0; k<vEmptyList.size(); ++k) {
            size_t i=vEmptyList[k];
            if (vExistList[k]) {
              vector<CJPDAFilter::RangeInterval> h; //空の範囲
              auto Result = vFilterSorted[i]->CalcP(h);
              dMultiply *= Result.first;
              Result.first = dBetaTotal;
              vBeta3[i].push_back(Result);
            }
            else {
              //temp
              if (!vBeta3[i].empty()) {
                cout << "Something Wrong, Beta3 not empty" << endl;
              }
              vBeta3[i].push_back(make_pair(dBetaTotal, -1));
            }
          }
          vector<int> vFilterStatus; //確認用．0:普通 1:存在するが見えてない 2:存在しない
          for (size_t i=0; i<vFilterSorted.size(); ++i) {
            auto it1 = std::find(vEmptyList.begin(), vEmptyList.end(), i);
            if (it1 == vEmptyList.end()) {
              dMultiply *= vFilterSorted[i]->GetExistanceRate();
              vFilterStatus.push_back(0);
            }
            else {
              size_t n=it1-vEmptyList.begin();
              if (vExistList[n]) {
                dMultiply *= vFilterSorted[i]->GetExistanceRate();
                vFilterStatus.push_back(1);
              }
              else {
                dMultiply *= (1.0-vFilterSorted[i]->GetExistanceRate());
                vFilterStatus.push_back(2);
              }
            }
          }


          for (size_t i=0; i<vBeta3.size(); ++i) {
            for (size_t j=0; j<vBeta3[i].size(); ++j) {
              vBeta3[i][j].first *= dMultiply;
            }
          }
          for (size_t i=0; i<vBeta3.size(); ++i) {
            vBeta[i].insert(vBeta[i].end(), vBeta3[i].begin(), vBeta3[i].end());
          }

          if (_CurrentConfig._bShowDebugString) {
            cout << "Muliply = " << dMultiply << endl;
            for (size_t i=0; i<vBeta3.size(); ++i) {
              cout << "#" << i << " ID=" << vFilterSorted[i]->GetPFID();
              if (vFilterStatus[i] == 1) cout << " (Not Found)";
              if (vFilterStatus[i] == 2) cout << " (Not Exist)";
              cout << endl;
              for (size_t j=0; j<vBeta3[i].size(); ++j) {
                cout << " Hypo#" << j << " Beta: [" << vBeta3[i][j].first << ":" << vBeta3[i][j].second  <<"]"<< endl;
              }
            }
          }


        }

      }
      else {
        //全部存在する仮定でOK
        double dNotFoundMultiply = 1.0;
        for (auto it=vEmptyList.begin(); it!=vEmptyList.end(); ++it) {
          vector<CJPDAFilter::RangeInterval> h; //空の範囲
          auto Result = vFilterSorted[*it]->CalcP(h);
          dNotFoundMultiply *= Result.first ;
          Result.first = dBetaTotal;
          vBeta2[*it].push_back(Result);
        }
        for (size_t i=0; i<vBeta2.size(); ++i) {
          for (size_t j=0; j<vBeta2[i].size(); ++j) {
            vBeta2[i][j].first *= dNotFoundMultiply;
          }
        }
        for (size_t i=0; i<vBeta2.size(); ++i) {
          vBeta[i].insert(vBeta[i].end(), vBeta2[i].begin(), vBeta2[i].end());
        }
        if (_CurrentConfig._bShowDebugString) {
          cout << "Muliply = " << dNotFoundMultiply << endl;
          for (size_t i=0; i<vBeta2.size(); ++i) {
            cout << "#" << i << " ID=" << vFilterSorted[i]->GetPFID() << endl;
            for (size_t j=0; j<vBeta2[i].size(); ++j) {
              cout << " Hypo#" << j << " Beta: " << vBeta2[i][j].first << endl;
            }
          }
        }
      }

    }

    double dTime3 = mt1.elapsed(); mt1.restart();

    for (size_t i=0; i<vBeta.size(); ++i) {
      auto &v1 = vBeta[i];
      double dTotal = 0;
      for (size_t j=0; j<v1.size(); ++j) {
        dTotal+=v1[j].first;
      }
      for (size_t j=0; j<v1.size(); ++j) {
        v1[j].first/=dTotal;
      }
      if (_CurrentConfig._bShowDebugString) {
        cout << "i=" << i << " Total=" << dTotal << endl;
      }
      if (dTotal == 0) {
        if (_CurrentConfig._bShowDebugString) {
          ccout ->SetColor(ColorCout::eRed);
          ccout << " something wrong BetaTotal=0 Filter #" << vFilterSorted[i]->GetPFID() << endl;
          ccout << " Beta Size: " << v1.size() << endl;
          for (size_t j=0; j<v1.size(); ++j) {
            ccout << "  " << v1[j].first << "/" << v1[j].second << ", ";
          }
          ccout << endl;          
          for (size_t j=0; j<v1.size(); ++j) {
            v1[j].first = 1.0/v1.size();
          }
        }
      }
      vFilterSorted[i]->Update(_CurrentFilterData, v1);
    }
    double dTime4 = mt1.elapsed(); mt1.restart();

    if (_bUseJIPDA) {
      //存在確率アップデート
      for (size_t i=0; i<vBeta.size(); ++i) {
        double dExistTotal = 0;
        double dTotal = 0;
        for (size_t k=0; k<vBeta[i].size(); ++k) {
          dTotal += vBeta[i][k].first;
          if (vBeta[i][k].second >= 0) {
            dExistTotal += vBeta[i][k].first;
          }
        }
        vFilterSorted[i]->SetExistanceRate(dExistTotal);
        if (_CurrentConfig._bShowDebugString) {
          cout << "Filter#" << vFilterSorted[i]->GetPFID() << " Rate: " << dExistTotal << " " << dTotal << endl;
        }
      }
    }

    //最尤仮説を_vBestHypoDivLinesに格納
    if (!vpBestHypos.empty()) {
      boost::shared_ptr<SHypothesis> pBestofBest;
      for (size_t i=0; i<vpBestHypos.size(); ++i) {
        if (!pBestofBest && vpBestHypos[i]) pBestofBest = vpBestHypos[i];
        else if (pBestofBest && vpBestHypos[i] && (pBestofBest->_dLikelihood < vpBestHypos[i]->_dLikelihood)) {
          pBestofBest = vpBestHypos[i];
        }
      }
      if (pBestofBest && (pBestofBest->_vCorrespondance.size() > 1)) {
        for (size_t i=0; i<pBestofBest->_vCorrespondance.size()-1; ++i) {
          double n1 = pBestofBest->_vCorrespondance[i]._vRanges.front().upper();
          double n2 = pBestofBest->_vCorrespondance[i+1]._vRanges.front().lower();
          _vBestHypoDivLines.push_back( (n1+n2)/2);
        }
      }
    }

    if (_CurrentConfig._bShowDebugString) {
      cout << "Beta" << endl;
      for (size_t i=0; i<vBeta.size(); ++i) {
        cout << "PF#" << vFilterSorted[i]->GetPFID() << " num=" << vBeta[i].size() << endl;
        for (size_t j=0; j<vBeta[i].size(); ++j) {
          cout << vBeta[i][j].first;
          if (vBeta[i][j].second >= 0) {
            cout << " (B" << vBeta[i][j].second << "=";
            const auto &rHypo = vFilterSorted[i]->GetHypoFromID(vBeta[i][j].second);
            if (!rHypo.empty()) {
              for (size_t k=0; k<rHypo.size(); ++k) {
                cout << "[" << rHypo[k].lower() << ":" << rHypo[k].upper() << "]";
              }
            }
            else {
              cout << "NotFound";
            }
            cout << ")" << endl;
          }
          else {
            cout << " (B=Disappered)" << endl;
          }
        }
      }
    }
    if (_CurrentConfig._bShowDebugString) {
      cout << "Frame end " << endl << endl;
    }
  }

  //各前景スキャン点についてスコアを求める
  //スコアが1に近いと既存移動体由来，0に近いと未発見移動体orFP
  _vExtractedPointsScore.resize(_pCurrentLaserData->GetPoints().size());
  std::fill(_vExtractedPointsScore.begin(), _vExtractedPointsScore.end(), 0); 
  for (auto it=_vpFilters.begin(); it!=_vpFilters.end(); ++it) {
    const auto &vBeta = (*it)->GetBeta();
    for (auto itBeta = vBeta.begin(); itBeta != vBeta.end(); ++itBeta) {
      if (itBeta->second >= 0) {
        const auto &rHypo = (*it)->GetHypoFromID(itBeta->second);
        double dVal = itBeta->first;
        for (auto itRange = rHypo.begin(); itRange != rHypo.end(); ++itRange) {
          for (int n=itRange->lower(); n<=itRange->upper(); ++n) {
            if (_vPointStatus[n] == 1) {
              _vExtractedPointsScore[n] += dVal;
            }
          }
        }
      }
    }
  }
  if (_CurrentConfig._bDumpDebugInfo) {
    DumpCSV();
  }
}

void CJPDATracker::CBetaCalculateHelper::GetNextPos(size_t n, std::vector<size_t> &rvPos) {
  rvPos = vNexts[_vRanges[n].upper()];
}

void CJPDATracker::ConstructHypo(vector<size_t> vHypo, const std::vector<boost::shared_ptr<CJPDATracker::CBetaCalculateHelper> > &rvpBeta,  vector<boost::shared_ptr<CJPDATracker::SHypothesis2> > &rvResults) {

  if (vHypo.size() == rvpBeta.size()) {
    double dP = 1;
    for (size_t i=0; i<rvpBeta.size(); ++i) {
      dP *= rvpBeta[i]->GetP()[vHypo[i]];
    }
    boost::shared_ptr<CJPDATracker::SHypothesis2> pResult(new CJPDATracker::SHypothesis2());
    pResult->_dLikelihood = dP;
//    pResult->_vnPos = vHypo;
    for (auto it=vHypo.begin(); it!=vHypo.end(); ++it) {
      pResult->_vnPos.push_back(*it);
    }
    rvResults.push_back(pResult);
    return;
  }

  vector<size_t> vNext;
  if (vHypo.empty()) {
    for (size_t i=0; i<rvpBeta[0]->GetP().size(); ++i) {
      vNext.push_back(i);
    }
  }
  else {
    const auto &pBeta = rvpBeta[vHypo.size()-1];
    size_t nPos = vHypo.back();
    pBeta->GetNextPos(nPos, vNext);
  }
  for (auto it=vNext.begin(); it!=vNext.end(); ++it) {
    vector<size_t> vHypo2 = vHypo;
    vHypo2.push_back(*it);
    ConstructHypo(vHypo2, rvpBeta, rvResults);
  }
}


boost::shared_ptr<CJPDATracker::SHypothesis> CJPDATracker::CalcBetaUsingEnumeration(std::vector<boost::shared_ptr<CBetaCalculateHelper> > &rvpHelper) {
  
  vector<boost::shared_ptr<SHypothesis2> > vHypothesis;
  vector<size_t> vTemp;

  vector<vector<double> > vBeta(rvpHelper.size());
  if (_bUseJIPDA) {
    ConstructHypo(vTemp, rvpHelper, vHypothesis);
  }
  else {
    ConstructHypo(vTemp, rvpHelper, vHypothesis);
  }

  for (size_t i=0; i<vBeta.size(); ++i) {
    vBeta[i] = std::vector<double>(rvpHelper[i]->GetRanges().size(), 0);
  }
  for (size_t k=0; k<vHypothesis.size(); ++k) {
    auto &pHypo = vHypothesis[k];
    for (size_t i=0; i<vBeta.size(); ++i) {
      vBeta[i][pHypo->_vnPos[i]] += pHypo->_dLikelihood;
    }
  }
  for (size_t i=0; i<vBeta.size(); ++i) {
    rvpHelper[i]->SetBeta(vBeta[i]);
  }

  size_t nMaxHypo = 0;
  double dMaxLikelihood = 0;
  for (size_t k=0; k<vHypothesis.size(); ++k) {
    if (vHypothesis[k]->_dLikelihood > dMaxLikelihood) {
      nMaxHypo = k;
      dMaxLikelihood = vHypothesis[k]->_dLikelihood;
    }
  }
  if (vHypothesis.empty()) {
    return boost::shared_ptr<SHypothesis>();
  }

  boost::shared_ptr<SHypothesis> pHypo(new SHypothesis);
  const auto &pHypo2 = vHypothesis[nMaxHypo];
  for (size_t i=0; i<rvpHelper.size(); ++i) {
    size_t nPos = pHypo2->_vnPos[i];
    SHypothesis::SCorrespondance Corr;
    Corr._pFilter = rvpHelper[i]->GetFilter();
    Corr._vRanges.push_back(rvpHelper[i]->GetRanges()[nPos]);
    pHypo->_vCorrespondance.push_back(Corr);
  }
  pHypo->_dLikelihood = pHypo2->_dLikelihood;

  return pHypo;
}

bool operator<(const CJPDATracker::SHypothesis2& r1, const CJPDATracker::SHypothesis2& r2) {
  return (r1._dLikelihood < r2._dLikelihood);
}

boost::shared_ptr<CJPDATracker::SHypothesis> CJPDATracker::CalcBestHypo(std::vector<boost::shared_ptr<CBetaCalculateHelper> > &rvpBeta) {

  priority_queue<SHypothesis2> HypoQueue;
  const auto &rvFirst = rvpBeta.front()->GetP();
  for (size_t j=0; j<rvFirst.size(); ++j){
    SHypothesis2 s; 
    s._dLikelihood=rvFirst[j];
    s._vnPos.push_back(j);
    HypoQueue.push(s);
  }

  SHypothesis2 BestHypo;
  BestHypo._dLikelihood = 0;

  while(true) {
    if (HypoQueue.empty()) break;
    auto Current = HypoQueue.top();
    HypoQueue.pop();
    if(Current._vnPos.size() == rvpBeta.size()) {     
      if (Current._dLikelihood > BestHypo._dLikelihood) {
        BestHypo = Current;
      }
    }
    else {
      if (Current._dLikelihood < BestHypo._dLikelihood) {
        break;
      }
      vector<size_t> vNext;
      rvpBeta[Current._vnPos.size()-1]->GetNextPos(Current._vnPos.back(), vNext);
      const auto &rP = rvpBeta[Current._vnPos.size()]->GetP();

      for (size_t j=0; j<vNext.size(); ++j){
        SHypothesis2 s = Current;
        s._dLikelihood *= rP[vNext[j]];
        s._vnPos.push_back(vNext[j]);
        HypoQueue.push(s);
      }
    }
  }

  if ((BestHypo._vnPos.size() == rvpBeta.size())) {

    boost::shared_ptr<SHypothesis> pHypo(new SHypothesis);
    for (size_t i=0; i<rvpBeta.size(); ++i) {
      size_t nPos = BestHypo._vnPos[i];
      SHypothesis::SCorrespondance Corr;
      Corr._pFilter = rvpBeta[i]->GetFilter();
      Corr._vRanges.push_back(rvpBeta[i]->GetRanges()[nPos]);
      pHypo->_vCorrespondance.push_back(Corr);
    }
    pHypo->_dLikelihood = BestHypo._dLikelihood;
    return pHypo;
  }
  else {
    //仮説がありえなすぎて尤度が全部0になったとき
    return boost::shared_ptr<CJPDATracker::SHypothesis>();
  }

}


void CJPDATracker::DumpCSV() {

  boost::filesystem::path pDumpDir;
  if (!boost::filesystem::exists("dbg")) 
    boost::filesystem::create_directory("dbg");
  ostringstream ossDir; ossDir << "dbg/dbg-" << _nCurrentFrame;

  while(true) {
    pDumpDir = ossDir.str();
    if (!boost::filesystem::exists(pDumpDir)) 
      boost::filesystem::create_directory(pDumpDir);
    else {
      boost::filesystem::directory_iterator end;
      try {
        for( boost::filesystem::directory_iterator it(pDumpDir); it!=end; ++it ) {
          boost::filesystem::remove(*it);
        }
        break;
      }
      catch (std::exception &e) {
        cout << __FUNCTION__ << " " << e.what() << endl;
        ossDir << "-new";
      }
    }
  }
  for (int i=0; i<(int)_vpFilters.size(); ++i) {
    const auto &rvWeight = _vpFilters[i]->GetParticleWeights();
    const auto &rvDebug  = _vpFilters[i]->GetParticleDebugInfo();
    const auto &rvTotalWeight = _vpFilters[i]->GetWeights();
    const auto &rvBeta = _vpFilters[i]->GetBeta();

    map<int, vector<double> > mBeta;
    map<int, double> mBetaTotal;
    for (auto it=rvBeta.begin(); it!=rvBeta.end(); ++it) {
      mBeta[it->second].push_back(it->first);
      mBetaTotal[it->second] += it->first;
    }

    double dTotal = 0;
    for (auto it= mBetaTotal.begin(); it!=mBetaTotal.end(); ++it) {
      dTotal += it->second;
    }

    if (rvWeight.empty()) {
      continue;
    }
    {
      ostringstream oss1; oss1 << "p" << _nCurrentFrame << "-" << _vpFilters[i]->GetPFID() << ".csv";
      ostringstream oss2; oss2 << "p" << _nCurrentFrame << "-" << _vpFilters[i]->GetPFID() << "-Hypo.csv";
      boost::filesystem::ofstream ofs1(pDumpDir/oss1.str());
      boost::filesystem::ofstream ofs2(pDumpDir/oss2.str());
      ofs1 << "PNum,TotalWeight";
      for (int nHypo=0; nHypo<(int)rvWeight.begin()->size(); ++nHypo) {
        ofs1 << ",NWeight" << nHypo;
        ofs1 << ",WWeight" << nHypo;
        for (size_t i=0; i<(size_t)CJPDAFilter::SLaserStatus::eInRangeTotal+(size_t)CJPDAFilter::SLaserStatus::eOutofRangeTotal; ++i) {
          ofs1 << "," << CJPDAFilter::SLaserStatus::GetLaserStatusStr(i);
        }
        ofs1 << ",RMin" << nHypo;
        ofs1 << ",RMax" << nHypo;
      }
      ofs1 << endl;

      for (int nPF=0; nPF<(int)rvWeight.size(); ++nPF) {
        ofs1 << nPF << "," << rvTotalWeight[nPF];
        for (int nHypo=0; nHypo<(int)rvWeight.begin()->size(); ++nHypo) {
          ofs1 << ", " << rvWeight[nPF][nHypo];
          ofs1 << ", " << rvWeight[nPF][nHypo]*mBetaTotal[nHypo];
          const auto &rD = rvDebug[nPF][nHypo];
          for (size_t i=0; i< rD.vnPointStatus.size(); ++i) {
            ofs1 << "," << rD.vnPointStatus[i];
          }
          ofs1 << "," << rD.nRangeMin << "," << rD.nRangeMax;
        }
        ofs1 << endl;
      }
      /*
      Betaは範囲が重複する．異なる並び順で同じ範囲が出てくるため．
      なお，rvWeight.begin()->size()は範囲の個数．Betaの個数より少ない．
      */
      ofs2 << "HypoNo,Range,BetaTotal,Beta0,..." << endl;
      for (int nHypo=0; nHypo<(int)rvWeight.begin()->size(); ++nHypo) {
        try {
          const auto &rHypo = _vpFilters[i]->GetHypoFromID(nHypo);
          if (mBeta.size() > (size_t)nHypo) {
            ofs2 << nHypo << ",";
            if (rHypo.empty()) ofs2 << "-";
            else {
              for (size_t i=0; i<rHypo.size(); ++i) {
                ofs2 << "[" << rHypo[i].lower() << ":" << rHypo[i].upper() << "] ";
              }
            }
            const auto &rBeta1 = mBeta[nHypo];
            ofs2 << "," << mBetaTotal[nHypo];
            for (size_t i=0; i<rBeta1.size(); ++i) {
              ofs2 << "," << rBeta1[i];
            }
            ofs2 << endl;
          }
        }
        catch (std::exception &e) {
          cout << "no hypo: " << nHypo << " " << e.what() << endl;
        }
      }

      //消えた場合の仮説
      const auto &rBeta1 = mBeta[-1];
      ofs2 << "Removed,-," << mBetaTotal[-1];
      for (size_t i=0; i<rBeta1.size(); ++i) {
        ofs2 << "," << rBeta1[i];
      }
      ofs2 << endl;
      
      ofs2 << "Total,," << dTotal << endl;
    }
  }
}

bool CMovingObjectSameID2(const boost::shared_ptr<CMovingObject> &p1, int n) {
  return (p1->GetID() == n);
}

boost::shared_ptr<const STrackerResult> CJPDATracker::Proc(const LaserDataBuffer &rvLasers) {

  mmtimer mtTotal;
#ifdef MEASURE_TIME
  mmtimer mt; 
  vector<double> vdTimes;
  vector<string> vsDesc;
#endif
  boost::recursive_mutex::scoped_lock lk(_ProcMutex);
  auto *pResult = new SJPDATrackerResult();
  pResult->_Config = _CurrentConfig;

  try {
    if (rvLasers.empty() || rvLasers.back().empty()) {
      ostringstream oss;
      oss << __FUNCTION__ << "buffer empty" << endl;
      throw std::logic_error(oss.str());
    }
    //step0 最初に必要な処理 _pCurrentLaserData:最新のLRFデータ，_pBeforeLaserData:前フレームのLRFデータ
    bool bUpdated = PrepareTracking(rvLasers);
    if (!bUpdated) {
      ++_nCurrentFrame;
      return boost::shared_ptr<const STrackerResult>();
    }
    _dCurrentTime = _pCurrentLaserData->GetTime();
    pResult->_nFrame = _nCurrentFrame;
    pResult->_dCurrentTime = _pCurrentLaserData->GetTime();
    if (_CurrentConfig._bShowDebugString) {
      ccout->SetColor(ColorCout::eGreen);
      ccout << endl;
      ostringstream oss;
      oss.setf(ios::fixed, ios::floatfield);
      oss << "Current Frame:" << _nCurrentFrame << " Time:" << setprecision(2) << _pCurrentLaserData->GetTime();
      ccout << oss.str() << endl;
      ccout << "Process Start" << endl;
    }
    //step1 候補領域の抽出 _vExtractedIDs, _vExtractedPointsに結果格納
    ExtractMovingPoints();
#ifdef MEASURE_TIME
    vdTimes.push_back(mt.elapsed()); mt.restart(); vsDesc.push_back("Extract");
#endif
    if (_bPerformTracking && (_CurrentConfig._nNoProcFrame < _nCurrentFrame)) {
      //step2 PFをPredict
      PredictPF();
#ifdef MEASURE_TIME
      vdTimes.push_back(mt.elapsed()); mt.restart(); vsDesc.push_back("Predict");
#endif
      //step3 仮説生成＆PFをUpdate
      SolveJPDA();
#ifdef MEASURE_TIME
      vdTimes.push_back(mt.elapsed()); mt.restart(); vsDesc.push_back("Hypo");
#endif
      //step4 追跡終了したPFを消去
      RemoveTerminatedPF();
#ifdef MEASURE_TIME
      vdTimes.push_back(mt.elapsed()); mt.restart(); vsDesc.push_back("Remove");
#endif
      //step5 候補領域をクラスタリング _vExtractedClustersに結果格納
      MakeCluster();
#ifdef MEASURE_TIME
      vdTimes.push_back(mt.elapsed()); mt.restart(); vsDesc.push_back("MkCluster");
#endif
      //step6 移動体の候補初期化
      InitializeNewObjects();
#ifdef MEASURE_TIME
      vdTimes.push_back(mt.elapsed()); mt.restart(); vsDesc.push_back("InitNew");
#endif
      /*
      if (_pSVMDataProcessing) {
        _pSVMDataProcessing->SetLRFData(_pCurrentLaserData, _vPointStatus);
        _pSVMDataProcessing->SavePFData(_vpFilters);
        for (size_t k=0; k<_vGroup.size(); ++k) {
          _pSVMDataProcessing->SaveGroupData(_vGroup[k]->GetFilters(), _vGroup[k]->GetGroupRangeOrg().lower(), _vGroup[k]->GetGroupRangeOrg().upper());
        }
      }
        */
    }
    pResult->_vExtractedPoints = _vExtractedPoints;
    pResult->_vExtractedClusters = _vExtractedClusters;
    pResult->_vBestHypoDivLines = _vBestHypoDivLines;
    pResult->_vTempPFResults.clear();
    pResult->_vPointStatus = _vPointStatus;
    for (auto it=_vpFilters.begin(); it!=_vpFilters.end(); ++it) {
      string sType = "";
      if (dynamic_cast<CJPDAEllipseFilter*>(it->get())) {
        sType = "ellipsecylinder";
      }
      else if (dynamic_cast<CJPDARectangleFilter*>(it->get())) {
        sType = "rectangle";
      }
      else {
        sType = "cylinder";
      }
      boost::shared_ptr<SPFStatus> pPFResult(new SPFStatus);
      pPFResult->_vAverage = (*it)->GetResult();
      pPFResult->_vParticles = (*it)->GetParticles(true);
      pPFResult->_vParticlesPredict = (*it)->GetParticles(false);
      pPFResult->_vdLikelihood = (*it)->GetWeights();
//      pPFResult->_vParticleCenters = (*it)->GetParticleCenters();
      pPFResult->_nClusterID = (*it)->GetPFID();
//      pPFResult->_AngleRange = (*it)->GetAngleRange();
      pPFResult->_sType = sType;
      pResult->_vTempPFResults.push_back(pPFResult);      
      int nID = (*it)->GetPFID();
      boost::shared_ptr<CMovingObject> pMO;
      if (_pCurrentResult) {
        auto itFound = std::find_if(_pCurrentResult->_vpObjects.begin(), _pCurrentResult->_vpObjects.end(), boost::bind(&CMovingObjectSameID2, _1, nID));
        if (itFound == _pCurrentResult->_vpObjects.end()) {
          pMO.reset(new CMovingObject(sType, nID));
        }
        else {
          pMO = (*itFound)->Clone();
        }
      }
      else {
        pMO.reset(new CMovingObject(sType, nID));
      }

      if (sType == "cylinder") {
        auto s1 = new SCylinderMovingObjectStatus;
        s1->_vPos(0) = (*it)->GetResult()(0);
        s1->_vPos(1) = (*it)->GetResult()(1);
        s1->_vVel(0) = (*it)->GetResult()(2);
        s1->_vVel(1) = (*it)->GetResult()(3);
        s1->_dRadius = (*it)->GetResult()(4);
        s1->_dMinHeight = 0;
        s1->_dMaxHeight = 1800;
        s1->_dExistenceRate = (*it)->GetExistanceRate();
        s1->_sStatus = "";
        pMO->AddCurrentStatus(boost::shared_ptr<SMovingObjectStatus>(s1), _nCurrentFrame);
        pResult->_vpObjects.push_back(pMO);

      }
      else if (sType == "ellipsecylinder") {
        auto s1 = new SEllipseCylinderMovingObjectStatus;
        s1->_vPos(0) = (*it)->GetResult()(0);
        s1->_vPos(1) = (*it)->GetResult()(1);
        s1->_vVel(0) = (*it)->GetResult()(2);
        s1->_vVel(1) = (*it)->GetResult()(3);
        s1->_dR1 = (*it)->GetResult()(4);
        s1->_dR2 = (*it)->GetResult()(5);
        s1->_dMinHeight = 0;
        s1->_dMaxHeight = 1800;
        s1->_dExistenceRate = (*it)->GetExistanceRate();
        s1->_sStatus = "";
        pMO->AddCurrentStatus(boost::shared_ptr<SMovingObjectStatus>(s1), _nCurrentFrame);
        pResult->_vpObjects.push_back(pMO);
      }
      else if (sType == "rectangle") {

        auto s1 = new SCuboidMovingObjectStatus;
        s1->_vPos(0) = (*it)->GetResult()(0);
        s1->_vPos(1) = (*it)->GetResult()(1);
        s1->_vVel(0) = (*it)->GetResult()(2);
        s1->_vVel(1) = (*it)->GetResult()(3);
        s1->_dLength = (*it)->GetResult()(4);
        s1->_dWidth  = (*it)->GetResult()(5);
        s1->_dMaxHeight = 1800;
        s1->_dExistenceRate = (*it)->GetExistanceRate();
        s1->_sStatus = "";
        pMO->AddCurrentStatus(boost::shared_ptr<SMovingObjectStatus>(s1), _nCurrentFrame);
        pResult->_vpObjects.push_back(pMO);
      }
    }
    pResult->_dStepTime = _dStepTime;
    pResult->_dProcTime = mtTotal.elapsed();

#ifdef MEASURE_TIME
    vdTimes.push_back(mt.elapsed()); mt.restart(); vsDesc.push_back("UpdateResult");
    if (pResult->_dProcTime > _dStepTime) {
//    if (pResult->_dProcTime > 0.05) {
      cout << "Too Heavy! ";
      cout << "CurFrame: " << _nCurrentFrame << endl;
      for (size_t i=0; i<vdTimes.size(); ++i) {
        cout << "Time #" << i << " "  << vdTimes[i] << "(" << vsDesc[i] << ") " << endl;
      }
      cout << "Total: " << pResult->_dProcTime << endl;
    }
#endif

    if ((_CurrentConfig._dProcResetTime > 0) && (pResult->_dProcTime > _CurrentConfig._dProcResetTime)) {
      ccout->SetColor(ColorCout::eRed);
      ccout << "Frame: " << _nCurrentFrame;
      ccout << " Proc Too Long: Reset Called" << endl;
      ResetAll();
    }

    if (_CurrentConfig._bShowDebugString) {
      ccout->SetColor(ColorCout::eGreen);
      ccout << "Current Frame:" << _nCurrentFrame << " Process End" << endl;
    }
  }
  catch (std::exception &e) {
    ccout->SetColor(ColorCout::eRed);
    ccout << "Proc Exception: " << e.what() << endl;
    WriteEventLog("Tracker Exception", e.what());
    if (_bCriticalError) {
      WriteEventLog("Critical Error", "");
    }
  }

  ++_nCurrentFrame;
  _pCurrentResult = boost::shared_ptr<const STrackerResult>(pResult);
  return _pCurrentResult;
}

void CJPDATracker::RemoveTerminatedPF() {

  auto it = _vpFilters.begin();
  while (it!=_vpFilters.end()) {

    CAngle dAngle;
    double dLen1, dLen2;
    boost::tie(dAngle, dLen1, dLen2) = (*it)->CalcErrorEllipse();

    if ((*it)->CrushWithLRF()) {
      ostringstream oss;
      oss << "PF #" << (*it)->GetPFID() << " CrushWithLRF";
      WriteEventLog("PF Terminated(CrushWithLRF)",oss.str());
      it = _vpFilters.erase(it);
    }
    
    else if ((*it)->GetExistanceRate() < _dRemoveExistanceRateThr) {
      ostringstream oss;
      oss << "PF #" << (*it)->GetPFID() << " GetExistanceRate Too Small" << (*it)->GetExistanceRate();
      WriteEventLog("PF Terminated(ExstanceRateTooSmall)",oss.str());
      it = _vpFilters.erase(it);
    }
    else if (dLen1 > _dRemoveDisparsionThr) {
      ostringstream oss;
      oss << "PF #" << (*it)->GetPFID() << " EllipseLen=" << dLen1 ;
      WriteEventLog("PF Terminated(disparsed)",oss.str());
      it = _vpFilters.erase(it);
    }
    else {
      ++it;
    }
  }

  //次に重なってしまったPFを探して消す
  //めんどいのでとりあえず長軸を半径とした円内にあったら消す
  //2個中消すのは歴史が浅い方(IDが小さい方)
  while(true) {
    int nTarget = -1;
    //消す対象がなくなるまで繰り返す
    for (size_t i=0; i<_vpFilters.size(); ++i) {
      auto pCar = dynamic_cast<CJPDARectangleFilter*>(_vpFilters[i].get());
      if (pCar) {
        for (size_t j=0; j<_vpFilters.size(); ++j) {
          if (i!=j) {
            auto pTargetCar = dynamic_cast<CJPDARectangleFilter*>(_vpFilters[j].get());
            if (pTargetCar) {
              if (HaveCarsCorrelation(pCar, pTargetCar)) {
                size_t n2=0;
                if (_vpFilters[i]->GetPFID() < _vpFilters[j]->GetPFID()) {
                  nTarget = j;
                  n2 = i;
                }
                else {
                  nTarget = i;
                  n2 = j;
                }
                ostringstream oss;
                oss << "PF(Car) #" << _vpFilters[nTarget]->GetPFID() << " near to: #" << _vpFilters[n2]->GetPFID();
                WriteEventLog("PF Terminated(car too near)",oss.str());
                goto end;

              }
            }
          }
        }
      }
      else {
        for (size_t j=0; j<_vpFilters.size(); ++j) {
          if (i!=j) {
            auto pTargetCar = dynamic_cast<CJPDARectangleFilter*>(_vpFilters[j].get());
            if (pTargetCar) {
              CPolygonalRegion Region;
              pTargetCar->GetResultPolygonal(Region);
              double dX1 = _vpFilters[i]->GetResult()(0);
              double dY1 = _vpFilters[i]->GetResult()(1);
              if (Region.IsPointInner(dX1, dY1)) {
                nTarget = i;
                ostringstream oss;
                oss << "PF #" << _vpFilters[nTarget]->GetPFID() << " within Car#" << pTargetCar->GetPFID();
                WriteEventLog("PF Terminated(too near to car)",oss.str());
                goto end;
              }
              else {
              }
            }
            else {
              double dX1 = _vpFilters[i]->GetResult()(0);
              double dY1 = _vpFilters[i]->GetResult()(1);
              double dX2 = _vpFilters[j]->GetResult()(0);
              double dY2 = _vpFilters[j]->GetResult()(1);

              double dR = max(max(max(_vpFilters[i]->GetResult()(4), _vpFilters[i]->GetResult()(5)), _vpFilters[j]->GetResult()(4)), _vpFilters[j]->GetResult()(5));
              dR *= 0.5;

              CAngle dA1 = FastMath::table_atan2(_vpFilters[i]->GetResult()(3), _vpFilters[i]->GetResult()(2));
              CAngle dA2 = FastMath::table_atan2(_vpFilters[j]->GetResult()(3), _vpFilters[j]->GetResult()(2));
              double dAngleDiff = abs((dA1-dA2).get_deg());

              double dDist = (dX1-dX2)*(dX1-dX2)+(dY1-dY2)*(dY1-dY2);
              if (dDist < dR*dR) {
                if (dAngleDiff<30.0) {
                  int n2 = 0;
                  if (_vpFilters[i]->GetPFID() < _vpFilters[j]->GetPFID()) {
                    nTarget = j;
                    n2 = i;
                  }
                  else {
                    nTarget = i;
                    n2 = j;
                  }
                  ostringstream oss;
                  oss << "PF #" << _vpFilters[nTarget]->GetPFID() << " near to: #" << _vpFilters[n2]->GetPFID() << " Dist=" << sqrt(dDist);
                  WriteEventLog("PF Terminated(too near)",oss.str());
                  goto end;
                }
              }
            }
          }
        }
      }
    }
    end:

    if (nTarget < 0) break;
    auto it=_vpFilters.begin();
    it+=nTarget;
    _vpFilters.erase(it);
  }
}

void CJPDATracker::WriteEventLog(const std::string &rsEvent, const std::string &rsDetail) {

  if (_CurrentConfig._bWriteEventLog) {
    ostringstream oss;
    oss << fixed << setprecision(6);
    oss << "Frame:" << _nCurrentFrame << ",(Time=" << _dCurrentTime << "),";
    oss << "Event:[" << rsEvent << "],";
    oss << "Detail:[" << rsDetail << "],";
    oss << "\n";
    _Logger.Log(oss.str());
  }
}

void CJPDATracker::ResetAll() {

  boost::recursive_mutex::scoped_lock lk(_ProcMutex);
  _nCurrentFrame = 0;
  _CurrentBuffer.clear();
  _vpFilters.clear();
  _vGroup.clear();
  _vGroupFirst.clear();

  WriteEventLog("reset", "reset called");

}

void CJPDATracker::SetPerformTracking(bool bPerformTrack) {

  boost::recursive_mutex::scoped_lock lk(_ProcMutex);
  ResetAll();

  _bPerformTracking  = bPerformTrack;
}
