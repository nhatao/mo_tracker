#include "StdAfx_MOTracking.h"
#include "TrackInitializationUsingGrid.h"
#include "GridSpace.h"
#include "FastMath.h"
#include "ColorCout.h"

using namespace std;

CTrackInitializationUsingGrid::CTrackInitializationUsingGrid() {

  _dDistThresh = DBL_MAX;
  _bUpdateBackGround = true;

  _dLRFOddsMax = 100;
  _dLRFOddsMin = -100;
  _dFreeThrLRF = 0.2;
  _dObstacleThrLRF = 0.9;

}

CTrackInitializationUsingGrid::~CTrackInitializationUsingGrid(void)
{
}

void CTrackInitializationUsingGrid::Initialize(double dXMax, double dXMin, double dYMax, double dYMin, double dXYGridSize) {
  _pGrid.reset(new CGridSpace(dXMax, dXMin, dYMax, dYMin, dXYGridSize, dXYGridSize));
//  ResetBackGround();
  _nXGridNum = _pGrid->GetXGridNum();
  _nYGridNum = _pGrid->GetYGridNum();
  _dMinX = _pGrid->GetXMin();
  _dMinY = _pGrid->GetYMin();
  _dMaxX = _pGrid->GetXMax();
  _dMaxY = _pGrid->GetYMax();
}

void CTrackInitializationUsingGrid::Initialize(const std::string &rsFileName){

  LoadBackGroundFromFile(rsFileName);
}
 
void CTrackInitializationUsingGrid::GetAreaInfo(double &rdXMin, double &rdXMax, double &rdYMin, double &rdYMax) const {

  rdXMin = _dMinX;
  rdXMax = _dMaxX;
  rdYMin = _dMinY;
  rdYMax = _dMaxY;
}


void CTrackInitializationUsingGrid::LoadBackGroundFromFile(const std::string &rsFileName) {

  cout << __FUNCTION__ << endl;

  ifstream ifs(rsFileName.c_str(), ios::in|ios::binary);
  if (!ifs.is_open()) {
    ostringstream oss;
    oss << "File not Exists!! :" << rsFileName;
    throw std::logic_error(oss.str().c_str());
  }

  string sLine;
  double dXMax, dXMin, dYMax, dYMin, dZMax, dZMin, dXYGridSize, dZGridSize;
  isread(ifs, dXMax);
  isread(ifs, dXMin);
  isread(ifs, dYMax);
  isread(ifs, dYMin);
  isread(ifs, dZMax);
  isread(ifs, dZMin);
  isread(ifs, dXYGridSize);
  isread(ifs, dZGridSize);

  //  cout << dXMax << " " <<  dXMin << " " <<   dYMax << " " <<   dYMin << " " <<   dZMax << " " <<   dZMin << " " <<   dXYGridSize << " " <<   dZGridSize << endl;

  _pGrid.reset(new CGridSpace(dXMax, dXMin, dYMax, dYMin, dXYGridSize, dXYGridSize));
  _nXGridNum = _pGrid->GetXGridNum();
  _nYGridNum = _pGrid->GetYGridNum();
  _dMinX = _pGrid->GetXMin();
  _dMinY = _pGrid->GetYMin();
  _dMaxX = _pGrid->GetXMax();
  _dMaxY = _pGrid->GetYMax();

  //グリッド読み込み
  SUnit* pGridUnit = _pGrid->GetObjects();
  SUnit* pGridUnitEnd = _pGrid->GetObjects()+_pGrid->GetTotalUnitNum();
  int n=0;
  while (pGridUnit != pGridUnitEnd) {
    pGridUnit->Load(ifs);
    ++pGridUnit;
    ++n;
  }

  cout << "loaded: " << n << endl;
}

void CTrackInitializationUsingGrid::SaveBackGroundToFile(const std::string &rsFileName) const {

  ofstream ofs;
//  ofs.open(rsFileName.c_str());
  ofs.open(rsFileName.c_str(), ios::out|ios::binary);

  //dummy
  double d=0;
  //サイズを保存
  oswrite(ofs, _pGrid->GetXMax());
  oswrite(ofs, _pGrid->GetXMin());
  oswrite(ofs, _pGrid->GetYMax());
  oswrite(ofs, _pGrid->GetYMin());
  oswrite(ofs, d);
  oswrite(ofs, d);
  oswrite(ofs, _pGrid->GetXGridSize());
  oswrite(ofs, d);

  //グリッド保存
  int n = 0;
  const SUnit* pGridUnit = _pGrid->GetObjects();
  const SUnit* pGridUnitEnd = _pGrid->GetObjects()+_pGrid->GetTotalUnitNum();
  while (pGridUnit != pGridUnitEnd) {
    pGridUnit->Dump(ofs);
    ++pGridUnit;
    ++n;
  }
}

void ResetGrid(SUnit& rUnit) {rUnit.dLikelihood = 0.5;} 
void CTrackInitializationUsingGrid::ResetBackGround() {

  _pGrid->ApplyAll(&ResetGrid);
}

void CTrackInitializationUsingGrid::SetOdds(double dObsOdds, double dFreeOdds) {
  _dLRFObsOdds = dObsOdds;
  _dLRFFreeOdds = dFreeOdds;

}

void CTrackInitializationUsingGrid::SetStatusThr(double dObsThr, double dFreeThr) {

  SetObstacleThrLRF(dObsThr);
  SetFreeThrLRF(dFreeThr);
}

void CTrackInitializationUsingGrid::SetOddsMinMax(double dOddsMin, double dOddsMax) {

  _dLRFOddsMax = std::max(dOddsMin, dOddsMax);
  _dLRFOddsMin = std::min(dOddsMin, dOddsMax);

}

bool CTrackInitializationUsingGrid::IsGridFree(const BoostVec &rV) {
  const SUnit *pUnit = _pGrid->GetOneObjectInCarteCoords(rV(0), rV(1));
  if (pUnit && pUnit->dLikelihood < _dFreeThrLRF) { //物体がない
    return true;
  }
  else return false;
}


void CTrackInitializationUsingGrid::SetLRFData2(const std::vector<std::vector<double> > &vLaserData, const std::vector<CCoordinates2D> &rvCoords, int nLaserNum, CAngle dDegBegin, CAngle dReso, double dLaserMaxLen) 
{


  auto *pCell = _pGrid->GetObjects();
  auto *pCellEnd = _pGrid->GetObjects()+_pGrid->GetTotalUnitNum();

  int nP360 = (int)(360/dReso.get_deg());
  double dAlpha = sqrt(2.0)*_pGrid->GetXGridSize()/2;

  if (vLaserData.size() != rvCoords.size()) {
    throw std::logic_error("rvpLRFData.size() != rvCoords.size()");
  }
  vector<double> vX;
  vector<double> vY;
  vector<double> vR;
  vector<int> vGridX;
  vector<int> vGridY;

  for (auto it=rvCoords.begin(); it != rvCoords.end(); ++it) {
    vX.push_back(it->GetPos()(0));
    vY.push_back(it->GetPos()(1));
    vR.push_back(it->GetRotation());
    vGridX.push_back(_pGrid->GetXPos(it->GetPos()(0)));
    vGridY.push_back(_pGrid->GetYPos(it->GetPos()(1)));
  }

  int nP1 = 0;
  int nP2 = 0;

  double dCS2 = _pGrid->GetXGridSize()/2;
  double dxc[4];
  double dyc[4];

  vector<int> vTemp;

  while (pCell != pCellEnd) {

    double dx = _pGrid->GridToPosX(pCell->aPos[0]);
    double dy = _pGrid->GridToPosY(pCell->aPos[1]);

    dxc[0] = dx + dCS2;
    dyc[0] = dy + dCS2;
    dxc[1] = dx - dCS2;
    dyc[1] = dy + dCS2;
    dxc[2] = dx - dCS2;
    dyc[2] = dy - dCS2;
    dxc[3] = dx + dCS2;
    dyc[3] = dy - dCS2;

    int nCX = pCell->aPos[0];
    int nCY = pCell->aPos[1];

    int nStatus = 0; //0:更新しない 1:減らす 2:増やす

    for (size_t i=0; i<vLaserData.size(); ++i) {
      const vector<double> & pData = vLaserData[i];
      
      int nLRFX = vGridX[i];
      int nLRFY = vGridY[i];

      int n1 = 0;
      int n2 = 0;

      if ((nCX > nLRFX) && (nCY > nLRFY)) {
        n1 = 3;
        n2 = 1;
      }
      else if ((nCX > nLRFX) && (nCY < nLRFY)) {
        n1 = 2;
        n2 = 0;
      }
      else if ((nCX < nLRFX) && (nCY < nLRFY)) {
        n1 = 1;
        n2 = 3;
      }
      else if ((nCX < nLRFX) && (nCY > nLRFY)) {
        n1 = 0;
        n2 = 2;
      }
      else if ((nCX == nLRFX) && (nCY > nLRFY)) {
        n1 = 3;
        n2 = 2;
      }
      else if ((nCX == nLRFX) && (nCY < nLRFY)) {
        n1 = 1;
        n2 = 0;
      }
      else if ((nCX > nLRFX) && (nCY == nLRFY)) {
        n1 = 2;
        n2 = 1;
      }
      else if ((nCX < nLRFX) && (nCY == nLRFY)) {
        n1 = 0;
        n2 = 3;
      }
      else {
        //同じ
      }
      int nPosMax;
      int nPosMin;
      {
        double xp = dxc[n1] - vX[i];
        double yp = dyc[n1] - vY[i];
        double rad = FastMath::table_atan2(yp, xp) - vR[i] - dDegBegin.get();
          
        double dRad2 = CAngle(rad).get_positive();
        int nPos = (int)round(dRad2/dReso.get());
        if (nPos == nP360) nPos = 0; //360回ったとき
        nPosMin = nPos; 
      }
      {
        double xp = dxc[n2] - vX[i];
        double yp = dyc[n2] - vY[i];
        double rad = FastMath::table_atan2(yp, xp) - vR[i] - dDegBegin.get();
        double dRad2 = CAngle(rad).get_positive();

        int nPos = (int)round(dRad2/dReso.get());
        if (nPos == nP360) nPos = 0; //360回ったとき
        nPosMax = nPos; 
      }

      double xp = dx - vX[i];
      double yp = dy - vY[i];
      double dist = xp*xp+yp*yp;
       
      nPosMin = max(0, nPosMin);
      nPosMax = min(nLaserNum-1, nPosMax);
      if (((nPosMax == nLaserNum-1) && (nPosMin >= nLaserNum)) || (nPosMax == 0)) {
        continue;
      }

      if (nPosMin > nPosMax) {
        if (nPosMax > 100) {
          //LRFのすぐ近く
          nStatus = 2;
        }
        nPosMin = 0;
      }
      for (int nPos = nPosMin; nPos <= nPosMax; ++nPos) {
        double dL = pData[nPos];
        if (dL == 0) dL = dLaserMaxLen;
        if ( abs(dL - sqrt(dist)) < dAlpha ) {
          //occupied
          nStatus = 2;
          break;
        }
        else if ( dL*dL > dist ) {
          //inner;
          nStatus = 1;
        }
      }
      if (nStatus == 2) break;
    }

    if (nStatus == 1) {
      ++nP2;
      pCell->dLikelihood -= _dLRFFreeOdds;
    }
    else if (nStatus == 2) {
      ++nP1;
      pCell->dLikelihood += _dLRFObsOdds;
    }
    pCell->dLikelihood = max(_dLRFOddsMin, pCell->dLikelihood);
    pCell->dLikelihood = min(_dLRFOddsMax, pCell->dLikelihood);
    ++pCell;

    if (nStatus == 0) {

    }
  }
}

bool CTrackInitializationUsingGrid::IsOutOfRange(const BoostVec &rv) {
  double dMargin = 1000;
  BoostVec v(2);
  v(0)=rv(0);
  v(1)=rv(1);
  return ((v(0) < _dMinX-dMargin) || (v(0) > _dMaxX+dMargin) || (v(1) < _dMinY-dMargin) || (v(1) > _dMaxY+dMargin));
}

void CTrackInitializationUsingGrid::Update(const LaserDataBuffer &Log, std::vector<int> &vnExtracted) {
  if (Log.empty()) return;

  const auto rvFirst = Log.back();
  if (rvFirst.size() != 1) {
    ESStreamException ess; ess << __FUNCTION__ << " LRFNum=" << rvFirst.size() << " not implemented";
    throw ess;
  }
  boost::shared_ptr<const CLaserData> pData = rvFirst.front();
  if (_bUpdateBackGround) {
    vector<vector<double> > vData;
    vData.push_back(pData->GetRawData());
    CCoordinates2D Co; 
    Co.SetPos(pData->GetCo()->GetX(), pData->GetCo()->GetY());
    Co.SetRotation(pData->GetCo()->GetYaw());
    vector<CCoordinates2D> vCo; vCo.push_back(Co);
    SetLRFData2(vData, vCo, pData->GetProperty()._nElemNum, pData->GetProperty()._dFirstAngle.get(), pData->GetProperty()._dReso.get(), pData->GetProperty()._dMaxRange);
  }

  int nLRFNum = 0;
  for (int i=0; i< (int)pData->GetPoints().size(); ++i) {
    const auto &pPos = pData->GetPoints()[i];
    if ((pPos->GetLocalX() !=0) || (pPos->GetLocalY() !=0)) {
      if (IsPointValid(pPos->GetWorldVec(), nLRFNum) && IsGridFree(pPos->GetWorldVec())) {
        vnExtracted.push_back(i);
      }
    }  
  }
}
