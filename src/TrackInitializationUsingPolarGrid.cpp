#include "StdAfx_MOTracking.h"
#include "TrackInitializationUsingPolarGrid.h"
#include "ColorCout.h"
#include <algorithm>
#include <boost/tuple/tuple.hpp>

using namespace std;

CTrackInitializationUsingPolarGrid::CTrackInitializationUsingPolarGrid(double dMaxLen, double dGridSize, size_t nLaserNum, size_t nStep)
{

  _dMaxLen = dMaxLen;
  _dGridSize = dGridSize;
  _nLaserNum = nLaserNum;
  _nStep = nStep;

  _nArrayX = 0;
  _nArrayY = 0;

  _dObsThr = log(0.99/(1-0.99));
  _dFreeThr = log(0.01/(1-0.01));
  _dLRFObsOdds = 1.0;
  _dLRFFreeOdds = 0.1;
  _dLRFOddsMax =  10.0;
  _dLRFOddsMin = -10.0;

  _dMinDistThr = 10;

  ReconstructArray();
}

void CTrackInitializationUsingPolarGrid::ReconstructArray() {

  _nArrayX = (size_t)ceil((double)_nLaserNum/_nStep);
  _nArrayY = (size_t)ceil(_dMaxLen/_dGridSize);
  _aArray.resize(_nArrayX*_nArrayY);
  std::fill(_aArray.begin(), _aArray.end(), 0);
}

CTrackInitializationUsingPolarGrid::~CTrackInitializationUsingPolarGrid(void)
{
  _aArray.clear();  
}

void CTrackInitializationUsingPolarGrid::ResetBackGround() {

  std::fill(_aArray.begin(), _aArray.end(), 0);
}

void CTrackInitializationUsingPolarGrid::SetOdds(double dObsOdds, double dFreeOdds) {
  _dLRFObsOdds = dObsOdds;
  _dLRFFreeOdds = dFreeOdds;
}

void CTrackInitializationUsingPolarGrid::SetStatusThr(double dObsThr, double dFreeThr) {
  _dObsThr = dObsThr;
  _dFreeThr = dFreeThr;
}

void CTrackInitializationUsingPolarGrid::SetOddsMinMax(double dOddsMin, double dOddsMax) {
  _dLRFOddsMax = max(dOddsMax, dOddsMin);
  _dLRFOddsMin = min(dOddsMax, dOddsMin);
}

void CTrackInitializationUsingPolarGrid::SetParams(double dObsThr, double dFreeThr, double dLRFObsOdds, double dLRFFreeOdds, double dLRFOddsMax, double dLRFOddsMin) {

  _dObsThr = dObsThr;
  _dFreeThr = dFreeThr;
  _dLRFObsOdds = dLRFObsOdds;
  _dLRFFreeOdds = dLRFFreeOdds;
  _dLRFOddsMax = dLRFOddsMax;
  _dLRFOddsMin = dLRFOddsMin;
}

void CTrackInitializationUsingPolarGrid::UpdateGrid (const std::vector<double>& aDist) {

  mmtimer mt;
  boost::mutex::scoped_lock lock(_ArrayMutex);
  if (_bUpdateBackGround) {
//    cout << "Free: " <<  _dLRFFreeOdds << " Obs: " << _dLRFObsOdds << endl;
    auto itFirst = _aArray.begin();

    for (size_t i=0; i<_nLaserNum; ++i) {

      double dDist = (double)aDist[i];
      size_t nPosX, nPosY;
      boost::tie(nPosX, nPosY) = GetArrayXY(i, dDist);
      size_t nFirstPos = GetLineFirstPos(nPosX);

      if ((dDist < _dMinDistThr) || (nPosY > _nArrayY-1)) {
        for (auto it = itFirst+nFirstPos; it != itFirst+nFirstPos+_nArrayY; ++it) {
          (*it) -= _dLRFFreeOdds/_nStep;
          (*it) = max(*it, _dLRFOddsMin);
        }
      }
      else {

        if (nPosY != 0) {         
          for (auto it = itFirst+nFirstPos; it != itFirst+nFirstPos+nPosY-1; ++it) {
            (*it) -= _dLRFFreeOdds/_nStep;
            (*it) = max(*it, _dLRFOddsMin);
          }
          auto pPoint = itFirst+nFirstPos+nPosY-1;
          (*pPoint) += _dLRFObsOdds/_nStep;
          (*pPoint) = min((*pPoint), _dLRFOddsMax);
          ++pPoint;
          (*pPoint) += _dLRFObsOdds/_nStep;
          (*pPoint) = min((*pPoint), _dLRFOddsMax);
          if (nPosY < _nArrayY-1) {
            ++pPoint;
            (*pPoint) += _dLRFObsOdds/_nStep;
            (*pPoint) = min((*pPoint), _dLRFOddsMax);
          }
        }
        else {
          auto pPoint = itFirst+nFirstPos;
          (*pPoint) += _dLRFObsOdds/_nStep;
          (*pPoint) = min((*pPoint), _dLRFOddsMax);
        }
      }
    }
  }
  else {

    int anCnt[10];
    memset(anCnt, 0, sizeof(int)*4);
    //奥だけ更新するモード
    //点なし:更新なし
    //遠距離:全体更新
    boost::mutex::scoped_lock lock2(_StatusMutex);
    _vCurrentStatusDebug.clear();
    auto itFirst = _aArray.begin();
    for (size_t i=0; i<_nLaserNum; ++i) {

      double dDist = (double)aDist[i];
      size_t nPosX, nPosY;
      boost::tie(nPosX, nPosY) = GetArrayXY(i, dDist);
      size_t nLineFirstPos = GetLineFirstPos(nPosX);
      size_t nArrayPos = GetArrayPos(nPosX, nPosY);

//      if (dDist < 10) continue;
//      if ((nPosY > _nArrayY-1)) {
      if ((dDist < _dMinDistThr) || (nPosY > _nArrayY-1)) {
        for (auto it = itFirst+nLineFirstPos; it != itFirst+nLineFirstPos+_nArrayY; ++it) {
          (*it) -= _dLRFFreeOdds/_nStep;
          (*it) = max(*it, _dLRFOddsMin);
        }
      }
      else {

        bool bUpdate = true;
        size_t nType = 0;
        if ((nPosY > _nArrayY-3)) {
          bUpdate = true;
          nType = 1;
        }
        else {
          double d1 = *(itFirst+nArrayPos-1);
          double d2 = *(itFirst+nArrayPos);
          double d3 = *(itFirst+nArrayPos+1);
          if (IsObstacle(d1) || IsObstacle(d2) || IsObstacle(d3)) {
            bUpdate = true;
            nType = 2;
          }
          else {
            //奥に物体があったらアップデートしない
            bool bOnlyFree = true;
            for (auto it = itFirst+nArrayPos+2; it != itFirst+nLineFirstPos+_nArrayY; ++it) {
              if (IsObstacle(*it)) {
                bUpdate = false;
                nType = 3;
                break;
              }
              if (IsUnknown(*it)) bOnlyFree = false;
            }
            if (bOnlyFree) {
              bUpdate = false;
              nType = 4;
            }
          }
        }

        SCurrentStatus s;
        s.nLaserPos = i;
        s.dLen = dDist;
        s.nCellStatus = nType;
        if (nType >= 5) cout << "piyoyo: " << nType << endl;
        s.nBeforeOcc = 0;
        _vCurrentStatusDebug.push_back(s);

        ++anCnt[nType];

        if (bUpdate) {

          if (nPosY != 0) {         
            for (auto it = itFirst+nLineFirstPos; it != itFirst+nLineFirstPos+nPosY-1; ++it) {
              (*it) -= _dLRFFreeOdds/_nStep;
              (*it) = max(*it, _dLRFOddsMin);
            }
            auto pPoint = itFirst+nLineFirstPos+nPosY-1;
            (*pPoint) += _dLRFObsOdds/_nStep;
            (*pPoint) = min((*pPoint), _dLRFOddsMax);
            ++pPoint;
            (*pPoint) += _dLRFObsOdds/_nStep;
            (*pPoint) = min((*pPoint), _dLRFOddsMax);
            if (nPosY < _nArrayY-1) {
              ++pPoint;
              (*pPoint) += _dLRFObsOdds/_nStep;
              (*pPoint) = min((*pPoint), _dLRFOddsMax);
            }
          }
          else {
            auto pPoint = itFirst+nLineFirstPos;
            (*pPoint) += _dLRFObsOdds/_nStep;
            (*pPoint) = min((*pPoint), _dLRFOddsMax);
          }
        }
      }
    }
  }
}


std::pair<unsigned int, unsigned int> CTrackInitializationUsingPolarGrid::GetArrayXY(size_t nLRFPos, double dDist) const {
  return std::make_pair(nLRFPos/_nStep, (size_t)floor(dDist/_dGridSize));
}

static const unsigned int eOccupied = 0;
static const unsigned int eFree     = 1;
static const unsigned int eUnknwon  = 2;


/*
 first: セルの状態
 second: 手前にOccupiedあり:1 なし:0
  Occupied                   ：静止物体
  Free，   手前にOccupiedなし：移動体
  Free，   手前にOccupiedあり：移動体(手前のOccupiedが移動した)

  Unknown，手前にOccupiedなし＋直後にOccupiedあり：境界領域=静止物体
  Unknown，手前にOccupiedあり＋奥にOccupiedなし：静止物体(手前のOccupiedにいた物体が移動した)
  Unknown，手前にOccupiedあり＋奥にOccupiedあり：移動物体(奥に壁があることがわかっている)


 nOccupiedStatusの内容
 0: 手前にOccなし 奥にOccなし -> Moving
 1: 手前にOccなし 奥にOccあり -> Moving
 2: 手前にOccなし 直後にOccあり -> Static

 4: 手前にOccあり 奥にOccなし -> Static
 5: 手前にOccあり 奥にOccあり -> Moving
 6: 手前にOccあり 直後にOccあり -> Static
*/
std::pair<unsigned int, unsigned int> 
  CTrackInitializationUsingPolarGrid::GetPointStatus(size_t nLRFPos, double dDist) const 
{

  if (dDist >= _dMaxLen) return make_pair(eUnknwon, 0);

  size_t nPosX, nPosY;
  boost::tie(nPosX, nPosY) = GetArrayXY(nLRFPos, dDist);

  size_t nLineFirst = GetLineFirstPos(nPosX);
  size_t nArrayPos = GetArrayPos(nPosX, nPosY);

  const double &d = _aArray[nArrayPos];
  auto itFirst = _aArray.begin();

  unsigned int nStatus;
  if (d < _dFreeThr) nStatus = eFree;
  else if (d > _dObsThr) nStatus = eOccupied;
  else nStatus = eUnknwon;

  bool bOccupiedBefore = false;
  if (nPosY > 1) {
    for (auto it = itFirst+nLineFirst; it != itFirst+nArrayPos-1; ++it) {
      if ( (*it) > _dObsThr) {
        bOccupiedBefore = true;
        break;
      }
    }
  }

  bool bOccupiedChokugo = false;
  unsigned int nLast = min(nPosY+3, _nArrayY);
  for (auto it = itFirst+nArrayPos+1; it != itFirst+nLineFirst+nLast; ++it) {
    if ( (*it) > _dObsThr) {
      bOccupiedChokugo = true;
      break;
    }
  }
  
  bool bOccpiedAfter = bOccupiedChokugo;
  if (!bOccpiedAfter) {
    for (auto it = itFirst+nArrayPos+1; it != itFirst+nLineFirst+_nArrayY; ++it) {
      if ( (*it) > _dObsThr) {
        bOccpiedAfter = true;
        break;
      }
    }
  }

  unsigned int nOccupiedStatus = 0;

  if (bOccupiedBefore) nOccupiedStatus+=4;
  if (bOccupiedChokugo) nOccupiedStatus+=2;
  else if (bOccpiedAfter) nOccupiedStatus+=1;

  return make_pair(nStatus, nOccupiedStatus);
}

bool CTrackInitializationUsingPolarGrid::IsMovingRegion(unsigned int nStatus, unsigned int nOccupiedStatus) const {
  if (nStatus == eFree) return true;
  if (nStatus == eOccupied) return false;
  if (nStatus == eUnknwon) {
    if ((nOccupiedStatus == 2) || (nOccupiedStatus == 6)) { //直後にOccがある場合はLRFの誤差
      return false;
    }
    if (nOccupiedStatus == 4) { //手前のOccupiedにいた物体が移動した
      return false;
    } 
    return true;
  }
  return false;
}

bool CTrackInitializationUsingPolarGrid::IsPointValid2(double dLen, const CAngle& dDeg, const CCoordinates2D &rLRFCo, size_t nLRFNo) const {

  if (dLen > _dMaxLen) { //極座標グリッドの領域外
    return false;
  }
  if (_vInvalidRegions.empty() && _vValidRegions.empty()) return true;
  
  _vGlobal.resize(2);
  _vGlobal(0) = dLen*cos(dDeg);
  _vGlobal(1) = dLen*sin(dDeg);
  rLRFCo.VectorLocalToGlobal(_vGlobal);

  return IsPointValid(_vGlobal, nLRFNo);
}

void CTrackInitializationUsingPolarGrid::Update(const LaserDataBuffer &Log, std::vector<int> &vnExtracted) {
  if (Log.empty()) return;

  const auto rvFirst = Log.back();
  if (rvFirst.size() != 1) {
    ESStreamException ess; ess << __FUNCTION__ << " LRFNum=" << rvFirst.size() << " not implemented";
    throw ess;
  }
  boost::shared_ptr<const CLaserData> pData = rvFirst.front();
  if (pData->GetRawData().size() != _nLaserNum) {
    ESStreamException ess; ess << __FUNCTION__ << " LaserNum Mismatch: " << pData->GetRawData().size() << "!=" << _nLaserNum;
    throw ess;
  }

  _dMinDistThr = rvFirst.front()->GetProperty()._dMinRange;

  const auto &rvArray = pData->GetRawData();
  UpdateGrid(rvArray);

  const auto pCo = pData->GetCo();
  CCoordinates2D Co(pCo->GetX(), pCo->GetY(), pCo->GetYaw());

  for (size_t i=0; i<_nLaserNum; ++i) {

    CAngle dDeg = pData->GetProperty().IndexToAngle(i);
    double dLen = rvArray[i];
    if ((dLen < pData->GetProperty()._dMaxRange) && (dLen >= pData->GetProperty()._dMinRange)) {
    
      unsigned int nStatus, nOccupiedStatus;
      boost::tie(nStatus, nOccupiedStatus) = GetPointStatus(i, dLen);           
      if (IsMovingRegion(nStatus, nOccupiedStatus)) {
        if (IsPointValid2(dLen, dDeg, Co, 0)) {
          vnExtracted.push_back((int)i);
        }
      }
    }
  }
}

#include "FastMath.h"

bool CTrackInitializationUsingPolarGrid::IsMoving(const BoostVec &rv, const SLRFProperty& rProp, boost::shared_ptr<const CCascadedCoords> pCo) {

  _dMinDistThr = rProp._dMinRange;
  double dDeg = FastMath::table_atan2(rv(1), rv(0));
  double dLen = FastMath::fast_hypot(rv(0), rv(1));
  int n = rProp.AngleToIndex(dDeg);
  //CCoordinates2D Co(pCo->GetX(), pCo->GetY(), pCo->GetYaw());
  static CCoordinates2D Co(0,0,0);

  unsigned int nStatus, nOccupiedStatus;
  boost::tie(nStatus, nOccupiedStatus) = GetPointStatus(n, dLen);           
  if (IsMovingRegion(nStatus, nOccupiedStatus)) {
    if (IsPointValid2(dLen, dDeg, Co, 0)) {
      return true;
    }
  }

  return false;
}

bool CTrackInitializationUsingPolarGrid::IsOutOfRange(const BoostVec &rv) {

  return false;
}

void CTrackInitializationUsingPolarGrid::GetAreaInfo(double &rdXMin, double &rdXMax, double &rdYMin, double &rdYMax) const {
  
  rdXMin = -_dMaxLen;
  rdXMax = _dMaxLen;
  rdYMin = -_dMaxLen;
  rdYMax = _dMaxLen;
}

void CTrackInitializationUsingPolarGrid::SaveBackGroundToFile(const std::string &rsFileName) const {
  ofstream ofs (rsFileName);

  ofs << "PolarGrid 1.0" << endl;
  ofs << _nArrayX << " " << _nArrayY << endl;

  for (size_t x=0; x<_nArrayX; ++x) {
    for (size_t y=0; y<_nArrayY; ++y) {
      ofs << _aArray[GetArrayPos(x,y)] << " ";
    }
    ofs << endl;
  }
}

void CTrackInitializationUsingPolarGrid::LoadBackGroundFromFile(const std::string &rsFileName) {

  ifstream ifs(rsFileName);
  if (!ifs.is_open()) {
    ESStreamException ess; ess << __FUNCTION__ << " file not found: " << rsFileName;
    throw ess;
  }

  string sLine;
  if (!getline(ifs, sLine)) {
    ESStreamException ess; ess << __FUNCTION__ << " file format invalid: " << sLine;
    throw ess;
  }

  if ((sLine.substr(0,13)  != "PolarGrid 1.0")) {
    ESStreamException ess; ess << __FUNCTION__ << " file format invalid2: " << sLine;
    throw ess;
  }
  
  if (!getline(ifs, sLine)) {
    ESStreamException ess; ess << __FUNCTION__ << " No Data";
    throw ess;
  }

  size_t nX, nY;
  istringstream iss(sLine);
  iss >> nX >> nY;

  if ((_nArrayX != nX) || (_nArrayY != nY)) {
    cout << __FUNCTION__ << " size mismatch: Current=" << _nArrayX << "*" << _nArrayY << " Loaded=" << nX << "*" << nY << endl;
    _nArrayX = nX;
    _nArrayY = nY;
    ReconstructArray();
  }


  string sLine2;
  sLine2.resize(4096*12);
  for (size_t x=0; x<_nArrayX; ++x) {
    if (!getline(ifs, sLine2)) {
      ESStreamException ess; ess << __FUNCTION__ << " Error at line " << x;
      throw ess;
    }
    istringstream iss2(sLine2);
    if (sLine2.size() > 4096*12) {
      cout << __FUNCTION__ << " buffer exausted: " << sLine2.size() << endl;
    }
    for (size_t y=0; y<_nArrayY; ++y) {
      unsigned int nPos = GetArrayPos(x,y);
      if (nPos >= _nArrayX*_nArrayY) {

        cout << "Something Wrong Pos=" << nPos << " x=" << x << " y=" << y << " MaxPox=" << _nArrayX*_nArrayY << endl;
      }
      else {
        iss2 >> _aArray[nPos];
      }
    }
  }

}

