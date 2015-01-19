#include "StdAfx_MOTracking.h"
#include "SlamMap.h"

#include "OutputDebugStream.h"
#ifdef _MSC_VER
#include <windows.h>
#pragma warning (disable:4996)
#endif
#include <cmath>
#include "Coordinates.h"
#include "MatrixFuncs.h"
#include <fstream>
#include "VoxelSpace.h"
#include "SlamMapSearch.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>

using namespace std;


double SUnit::dLikelihoodThresh = 0.9;


void InitSearchMapState(SSearchMapState &rState) {
  rState.nVotedNum = 0;
  rState.nLabel = -1;
  rState.State = SSearchMapState::UNKNOWN;
}

template <class T>
void RegisterNeighborsWithGnd18(T &rUnit, CVoxelSpaceTemplate<T>* pVoxel) {
  int a2N[][3] = {  {-1, 0, 0},{ 1, 0, 0},{ 0,-1, 0},{ 0, 1, 0},{ 0, 0, 1},{ 0, 0,-1},
                    {-1,-1, 0},{ 1,-1, 0},{ 0,-1, 1},{ 0, 1, 1},{-1, 0, 1},{-1, 0,-1},
                    {-1, 1, 0},{ 1, 1, 0},{ 0,-1,-1},{ 0, 1,-1},{ 1, 0, 1},{ 1, 0,-1},
  };
  RegisterNeighborsWithGnd(rUnit, pVoxel, a2N, 18);
}
template <class T>
void RegisterNeighborsWithGnd(T &rUnit, CVoxelSpaceTemplate<T>* pVoxel, int pNeighbors[][3], int nSize) {

  int i = rUnit.aPos[0];
  int j = rUnit.aPos[1];
  int k = rUnit.aPos[2];
  rUnit.vnNeighbors->clear();
  for(int n=0; n< nSize; ++n) {
    int pos = pVoxel->GetArrayPosWithCheck(i+(pNeighbors)[n][0],j+(pNeighbors)[n][1],k+(pNeighbors)[n][2]);
    if ( 
        (pos>0) && 
        (!((k==2) && ((k+(pNeighbors)[n][2])==1))) &&
        (!((k==1) && ((k+(pNeighbors)[n][2])==2))) 
      )
    {
//      rUnit.vpNeighbors->push_back(pVoxel->GetObjects()+pos);
      rUnit.vnNeighbors->push_back(pos);
    }
  }
}


CVoxelSpaceWithGround::CVoxelSpaceWithGround(double dXMax, double dXMin, double dYMax, double dYMin, double dZMax, double dZMin, double dXGridSize, double dYGridSize, double dZGridSize)
  : CVoxelSpace(dXMax, dXMin, dYMax, dYMin, dZMax, dZMin, dXGridSize, dYGridSize, dZGridSize)
{

  ApplyAll(boost::bind(RegisterNeighborsWithGnd18<SUnit>, _1, this));
}

void SetArraytoCoords(CCoordinates3D &rCo, const float* pos, const float* rot) {

  rCo.SetPos(pos[0], pos[1], pos[2]);
  Mat m;
  m.resize(3,3);
  for(int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      m(i,j)=rot[j+i*3];
    }
  }
  rCo.SetRot(m);
}

void SetCoordstoArray(const CCoordinates3D &rCo, float* pos, float* rot) {

  const Vec& vPos = rCo.GetPos();
  const Mat& mRot = rCo.GetRot();
  for(int i=0; i<3; ++i) {
    pos[i]=(float)vPos(i);
    for (int j=0; j<3; ++j) {
      rot[j+i*3] = (float)mRot(i, j);
    }
  }
}


CSlamMap::CSlamMap(double dXMax, double dXMin, double dYMax, double dYMin, double dZMax, double dZMin, double dXYGridSize, double dZGridSize)
{

  _pGrid = new CGridSpace(dXMax, dXMin, dYMax, dYMin, dXYGridSize, dXYGridSize);
  _pVoxel = new CVoxelSpaceWithGround(dXMax, dXMin, dYMax, dYMin, dZMax, dZMin, dXYGridSize, dXYGridSize, dZGridSize);
  _pSearchMap = new CGridSpaceTemplate<SSearchMapState>(dXMax, dXMin, dYMax, dYMin, dXYGridSize, dXYGridSize);
  _pGridForVision = new CGridSpaceTemplate<SSimpleUnit>(320, 0, 240, 0, 1, 1);
  _nXGridNum = _pVoxel->GetXGridNum();
  _nYGridNum = _pVoxel->GetYGridNum();
  _nZGridNum = _pVoxel->GetZGridNum();

  _dLRFObsOdds = 1;
  _dLRFFreeOdds = 0.1;
  _dLRFOddsMax = 100;
  _dLRFOddsMin = -100;

  InitParams();

  cout << "grid unit num  2: " << _pGrid->GetTotalUnitNum() << endl;
  cout << "voxel unit num 2: " << _pVoxel->GetTotalUnitNum() << endl;
  cout << "search unit num2: " << _pSearchMap->GetTotalUnitNum() << endl;

}

CSlamMap::CSlamMap(const std::string &rsFileName) {

  _pGrid = NULL;
  _pVoxel = NULL;
  _pGridForVision = NULL;
  _pSearchMap = NULL;

  LoadFromFile(rsFileName);
}

CSlamMap* CSlamMap::MakeCopyMap() {//中でnewする

  const CVoxelSpace *pVox = GetVoxel();
  double dXMax = pVox->GetXMax();
  double dXMin = pVox->GetXMin();
  double dYMax = pVox->GetYMax();
  double dYMin = pVox->GetYMin();
  double dZMax = pVox->GetZMax();
  double dZMin = pVox->GetZMin();
  double dXYGridSize = pVox->GetXGridSize();
  double dZGridSize  = pVox->GetZGridSize();

  CSlamMap *pMap = new CSlamMap(dXMax, dXMin, dYMax, dYMin, dZMax, dZMin, dXYGridSize, dZGridSize);
  (*pMap) = (*this);
  pMap->_pGrid = new CGridSpace(dXMax, dXMin, dYMax, dYMin, dXYGridSize, dXYGridSize);
  pMap->_pVoxel = new CVoxelSpaceWithGround(dXMax, dXMin, dYMax, dYMin, dZMax, dZMin, dXYGridSize, dXYGridSize, dZGridSize);
  pMap->_pGridForVision = new CGridSpaceTemplate<SSimpleUnit>(320, 0, 240, 0, 1, 1);
  pMap->_pSearchMap = new CGridSpaceTemplate<SSearchMapState>(dXMax, dXMin, dYMax, dYMin, dXYGridSize, dXYGridSize);

  pMap->_pGrid->Copy(*_pGrid);
  pMap->_pVoxel->Copy(*_pVoxel);
  pMap->_pGridForVision->Copy(*_pGridForVision);
  pMap->_pSearchMap->Copy(*_pSearchMap);

  return pMap;
}

bool CSlamMap::LoadFromFile(const std::string &rsFileName) {

  ifstream ifs(rsFileName.c_str(), ios::in|ios::binary);
  if (!ifs.is_open()) {
    dout << "File not Exists!! :" << rsFileName << endl;
    return false;
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

  cout << dXMax << " " <<  dXMin << " " <<   dYMax << " " <<   dYMin << " " <<   dZMax << " " <<   dZMin << " " <<   dXYGridSize << " " <<   dZGridSize << endl;


//  DeleteObjects();
  _pGrid = new CGridSpace(dXMax, dXMin, dYMax, dYMin, dXYGridSize, dXYGridSize);
  _pGridForVision = new CGridSpaceTemplate<SSimpleUnit>(320, 0, 240, 0, 1, 1);
  _pSearchMap = new CGridSpaceTemplate<SSearchMapState>(dXMax, dXMin, dYMax, dYMin, dXYGridSize, dXYGridSize);
  _pVoxel = new CVoxelSpaceWithGround(dXMax, dXMin, dYMax, dYMin, dZMax, dZMin, dXYGridSize, dXYGridSize, dZGridSize);

  cout << "grid unit num: " << _pGrid->GetTotalUnitNum() << endl;
  cout << "voxel unit num: " << _pVoxel->GetTotalUnitNum() << endl;
  cout << "search unit num: " << _pSearchMap->GetTotalUnitNum() << endl;

  _nXGridNum = _pVoxel->GetXGridNum();
  _nYGridNum = _pVoxel->GetYGridNum();
  _nZGridNum = _pVoxel->GetZGridNum();

//  InitParams();


//  return true;


  //グリッド読み込み
  SUnit* pGridUnit = _pGrid->GetObjects();
  SUnit* pGridUnitEnd = _pGrid->GetObjects()+_pGrid->GetTotalUnitNum();
  int n=0;
  while (pGridUnit != pGridUnitEnd) {
    pGridUnit->Load(ifs);
    ++pGridUnit;
    ++n;
  }
  cout << "loaded " << n << endl;

  //ボクセル読み込み
  pGridUnit = _pVoxel->GetObjects();
  pGridUnitEnd = _pVoxel->GetObjects()+_pVoxel->GetTotalUnitNum();
  while (pGridUnit != pGridUnitEnd) {
    pGridUnit->Load(ifs);
    ++pGridUnit;
  }
  cout << "aa" << endl;
  _pSearchMap->ApplyAll(&InitSearchMapState);
  cout << "bb" << endl;
  UpdateState();
  cout << "cc" << endl;

  return true;
}

void CSlamMap::SaveToFile(const std::string &rsFileName) const {

  ofstream ofs;
//  ofs.open(rsFileName.c_str());
  ofs.open(rsFileName.c_str(), ios::out|ios::binary);

  //サイズを保存
  oswrite(ofs, _pVoxel->GetXMax());
  oswrite(ofs, _pVoxel->GetXMin());
  oswrite(ofs, _pVoxel->GetYMax());
  oswrite(ofs, _pVoxel->GetYMin());
  oswrite(ofs, _pVoxel->GetZMax());
  oswrite(ofs, _pVoxel->GetZMin());
  oswrite(ofs, _pVoxel->GetXGridSize());
  oswrite(ofs, _pVoxel->GetZGridSize());

  //グリッド保存
  int n = 0;
  const SUnit* pGridUnit = _pGrid->GetObjects();
  const SUnit* pGridUnitEnd = _pGrid->GetObjects()+_pGrid->GetTotalUnitNum();
  while (pGridUnit != pGridUnitEnd) {
    pGridUnit->Dump(ofs);
    ++pGridUnit;
    ++n;
  }
  cout << "dumped " << n << endl;

  //ボクセル保存
  pGridUnit = _pVoxel->GetObjects();
  pGridUnitEnd = _pVoxel->GetObjects()+_pVoxel->GetTotalUnitNum();
  while (pGridUnit != pGridUnitEnd) {
    pGridUnit->Dump(ofs);
    ++pGridUnit;
  }
}

CSlamMap::~CSlamMap(void)
{
  DeleteObjects();
}

void CSlamMap::InitParams() {

  _pSearchMap->ApplyAll(InitSearchMapState);
  SetCurCoord(0,0,0);

  _dPlaneThr = 20;
  _dVisionObstacleThr = 20;
  _dLRFThr = 2;
  _dGridScore = 1.0;
  _dVoxelScore = 1.0;

  _nMinRegion = 10;
  _dRobotSize = 400;
  //  temp.resize(3);

  _dPOELRF = 0.999;
  _dPOENLRF = 0.15;
  _dPOEVision = 0.9;
  _dPOENVision = 0.3;
  _dFreeThrLRF = 0.2;
  _dObstacleThrLRF = 0.9;
  _dFreeThrVision = 0.2;
  _dObstacleThrVision = 0.95;

  _dBF = 28658.0;
  _dB  = 88.7244;
  _dXC = 160.852;
  _dYC = 119.370;

  _nCamXMin=40;
  _nCamYMin=6;
  _nCamXMax=280;
  _nCamYMax=234;

  _dVisionMaxLen = 2000;
  _nVisionNoiseThr = 400;
  _dVisionSameGroupDistThr = 2;

}

void CSlamMap::DeleteObjects() {

  delete _pGrid;
  delete _pVoxel;
  delete _pGridForVision;
  delete _pSearchMap;
}

void CSlamMap::SetCurCoord(double x, double y, double r, double v, double w) {

  _RobotCo.SetPos(x,y,0);
  _RobotCo.SetRPYAngle(r,0,0);
  _dRobotCurV = v;
  _dRobotCurW = w;
}

void CSlamMap::SetCurCoord(const CCoordinates2D &rCo, double v, double w) {

  SetCurCoord(rCo.GetPos()(0),rCo.GetPos()(1), rCo.GetRotation(), v, w);
}


void ClearUnit(SUnit &rUnit) {

  rUnit.nLabel = -1;
  rUnit.nVotedNum = 0;
  rUnit.dLikelihood=0.5;
  rUnit.dRAverage=128;
  rUnit.dGAverage=128;
  rUnit.dBAverage=128;
  rUnit.bChanged = false;
}


void CSlamMap::Clear() {

  _pVoxel->ApplyAll(&ClearUnit);
  _pGrid->ApplyAll(&ClearUnit);

//  int _nXGridNum = _pGrid->GetXGridNum();
//  int _nYGridNum = _pGrid->GetYGridNum();

  _pSearchMap->ApplyAll(InitSearchMapState);
  /*
  SSearchMapState state;
  for (int i=0; i<_nXGridNum; ++i) {
    for (int j=0; j<_nYGridNum; ++j) {
      _v2SearchMap[i][j] = state;
    }
  }
  */
  SetCurCoord(0,0,0);

}


void CSlamMap::ApplyFuncOnPath(int x1, int y1, int x2, int y2, SearchMapFunc& f1) const{

//  double dSum = 0;

  if (x1 == x2) {
    int ymin = min(y1,y2);
    int ymax = max(y1,y2);
    for (int y = ymin; y<=ymax; ++y) {
//      dSum += _v2SearchMap[x1][y].dDistanceFromObs;
//      f1(_v2SearchMap[x1][y]);
      f1(*_pSearchMap->GetOneObject(x1,y));
    }
    return;
  }
  if (y1 == y2) {
    int xmin = min(x1,x2);
    int xmax = max(x1,x2);
    for (int x = xmin; x<=xmax; ++x) {
//      dSum += _v2SearchMap[x][y1].dDistanceFromObs;
//      f1(_v2SearchMap[x][y1]);
      f1(*_pSearchMap->GetOneObject(x,y1));
    }
    return;
  }

  bool isXLong = ( abs((x1-x2)) > abs((y1-y2)));
  if (isXLong) {
    int xmin, ymin, xmax, ymax;
    if (x1>x2) {
      xmin = x2;
      ymin = y2;
      xmax = x1;
      ymax = y1;
    }
    else {
      xmin = x1;
      ymin = y1;
      xmax = x2;
      ymax = y2;
    }
    double dGrad = ((double)(ymax-ymin))/(xmax-xmin);
    double y = ymin;
    for (int x = xmin; x<=xmax; ++x) {
      int iy = (int)(round(y));
//      f1(_v2SearchMap[x][iy]);
      f1(*_pSearchMap->GetOneObject(x,iy));
//      dSum += _v2SearchMap[x][iy].dDistanceFromObs;
      y += dGrad;
    }
    return;
  }
  else {

    int xmin, ymin, xmax, ymax;
    if (y1>y2) {
      xmin = x2;
      ymin = y2;
      xmax = x1;
      ymax = y1;
    }
    else {
      xmin = x1;
      ymin = y1;
      xmax = x2;
      ymax = y2;
    }
    double dGrad = ((double)(xmax-xmin))/(ymax-ymin);
    double x = xmin;
    for (int y = ymin; y<=ymax; ++y) {
      int ix = (int)(round(x));
      f1(*_pSearchMap->GetOneObject(ix,y));
//      f1(_v2SearchMap[ix][y]);
//      dSum += _v2SearchMap[ix][y].dDistanceFromObs;
      x += dGrad;
    }
    return;
  }
}

bool CSlamMap::CheckObstacleGrid(int x1, int y1, int x2, int y2) const{

  if (x1 == x2) {
    int ymin = min(y1,y2);
    int ymax = max(y1,y2);
    for (int y = ymin; y<=ymax; ++y) {
//      if (_v2SearchMap[x1][y].IsOccupied()) {
      if (_pSearchMap->GetOneObject(x1,y)->IsOccupied()) {
        return true;
      }
    }
    return false;
  }
  if (y1 == y2) {
    int xmin = min(x1,x2);
    int xmax = max(x1,x2);
    for (int x = xmin; x<=xmax; ++x) {
      if (_pSearchMap->GetOneObject(x,y1)->IsOccupied()) {
//      if (_v2SearchMap[x][y1].IsOccupied()) {
        return true;
      }
    }
    return false;
  }

  bool isXLong = ( abs((x1-x2)) > abs((y1-y2)));

  if (isXLong) {
    int xmin, ymin, xmax, ymax;
    if (x1>x2) {
      xmin = x2;
      ymin = y2;
      xmax = x1;
      ymax = y1;
    }
    else {
      xmin = x1;
      ymin = y1;
      xmax = x2;
      ymax = y2;
    }
    double dGrad = ((double)(ymax-ymin))/(xmax-xmin);
    double y = ymin;
    for (int x = xmin; x<=xmax; ++x) {
      int iy = (int)(round(y));
      if (_pSearchMap->GetOneObject(x,iy)->IsOccupied()) {
//      if (_v2SearchMap[x][iy].IsOccupied()) {
        return true;
      }
      y += dGrad;
    }
  }
  else {

    int xmin, ymin, xmax, ymax;
    if (y1>y2) {
      xmin = x2;
      ymin = y2;
      xmax = x1;
      ymax = y1;
    }
    else {
      xmin = x1;
      ymin = y1;
      xmax = x2;
      ymax = y2;
    }
    double dGrad = ((double)(xmax-xmin))/(ymax-ymin);
    double x = xmin;
    for (int y = ymin; y<=ymax; ++y) {
      int ix = (int)(round(x));
      if (_pSearchMap->GetOneObject(ix,y)->IsOccupied()) {
//      if (_v2SearchMap[ix][y].IsOccupied()) {
        return true;
      }
      x += dGrad;
    }
  }
  return false;
}

template <class T>
inline void VoteWithBayesian(T& Unit, double POE, double POEN, bool bOnceATime=true) {
  if ( (!Unit.bChanged) || (!bOnceATime) ) {
    Unit.nVotedNum += 1;
    Unit.dLikelihood = POE*(Unit.dLikelihood)/(POE*(Unit.dLikelihood)+POEN*(1-Unit.dLikelihood));
    Unit.dLikelihood = min(0.999999, Unit.dLikelihood);
    Unit.dLikelihood = max(0.000001, Unit.dLikelihood);
    Unit.bChanged = true;
  }
  else  {
  }
}

void VoteRGB(SUnit& unit, int nVoteNum, unsigned char* pRGB) {
  unit.dRAverage = pRGB[0];
  unit.dGAverage = pRGB[1];
  unit.dBAverage = pRGB[2];
  unit.nVotedNum += nVoteNum;
  unit.dLikelihood += nVoteNum;
}

void VoteRGBWithBayesian(SUnit& unit, double POE, double POEN, unsigned char* pRGB, bool bOnceATime=true) {
  unit.dRAverage = pRGB[0];
  unit.dGAverage = pRGB[1];
  unit.dBAverage = pRGB[2];
  VoteWithBayesian(unit,POE,POEN,bOnceATime);
}


//LRFの距離データと、LRFの座標(ワールド座標)を渡す
void CSlamMap::SetLRFData(int *pLRFData, int nLaserNum, double dDegBegin, double dReso, float* pPos, float* pRot) {

  SetArraytoCoords(_TempCo, pPos, pRot);
  dReso = fabs(dReso); //temp eusとの互換性
  int nDistNum = (int)(360/dReso)+1;
  double *aDist = new double[nDistNum];
  for (int i=0; i<nDistNum; ++i) aDist[i]=-1;

  //物体のあるところに投票
  double dCurDeg = dDegBegin;
  Vec temp(3);
  for (int i=0; i<nLaserNum; ++i, dCurDeg+=dReso) {
    if (pLRFData[i] > 20) {
      double d = dCurDeg;
      if (d < 0) d+=360;
      if (d >= 360) d-=360;
      aDist[(int)round(d/dReso)] = pLRFData[i];
      temp(0) = pLRFData[i]*sin(Deg2Rad(dCurDeg));
      temp(1) = pLRFData[i]*cos(Deg2Rad(dCurDeg));
      temp(2) = 0;
      _TempCo.TransformVector(temp);
      if ( hypot(temp(0) - _RobotCo.GetPos()(0), temp(1) - _RobotCo.GetPos()(1)) > _dRobotSize) {
        _pGrid->ApplyCarteCoords(temp, boost::bind(&VoteWithBayesian<SUnit>, _1, _dPOELRF, _dPOENLRF, true));
      }
    }
    else {
    }
  }
  float dLRFR = (float)_TempCo.GetYaw();

  //物体のないところに投票
  for (int x=0; x<_nXGridNum; ++x) {
    for (int y=0; y<_nYGridNum; ++y) {
      double xp = _pGrid->GridToPosX(x) - pPos[0];
      double yp = _pGrid->GridToPosY(y) - pPos[1];
      double dist = sqrt(xp*xp+yp*yp);
      double deg = Rad2Deg(atan2(xp, yp) + dLRFR);
      if (deg < 0) deg+=360; if (deg > 360) deg-=360;
      int nDeg = (int)round(deg/dReso);
      if (aDist[nDeg] > dist) {
        _pGrid->ApplyGridCoords(x, y, boost::bind(&VoteWithBayesian<SUnit>, _1, 1-_dPOELRF, 1-_dPOENLRF, true));
      }
    }
  }
  delete[] aDist;
}

#include "Angle.h"
#include "FastMath.h"
#include "ColorCout.h"

void CSlamMap::SetLRFData2(const std::vector<std::vector<double> > &vLaserData, const std::vector<CCoordinates2D> &rvCoords,
                   int nLaserNum, double dDegBegin, double dReso, double dLaserMaxLen) {

  mmtimer mt;

  auto *pCell = _pGrid->GetObjects();
  auto *pCellEnd = _pGrid->GetObjects()+_pGrid->GetTotalUnitNum();

  double dRadBegin = Deg2Rad(dDegBegin);
  double dRadReso = Deg2Rad(dReso);

  int nP360 = (int)(360/dReso);
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
//        double rad = CAngle(atan2(yp, xp) - vR[i] - dRadBegin).get_positive();
//        double rad = CAngle(FastMath::table_atan2(yp, xp) - vR[i] - dRadBegin).get_positive();
        double rad = FastMath::table_atan2(yp, xp) - vR[i] - dRadBegin;
        while (rad >= 2*M_PI) rad -= 2*M_PI;
        while (rad < 0)       rad += 2*M_PI;

        int nPos = (int)round((rad/dRadReso));
        if (nPos == nP360) nPos = 0; //360回ったとき
        nPosMin = nPos; 
      }
      {
        double xp = dxc[n2] - vX[i];
        double yp = dyc[n2] - vY[i];
//        double rad = CAngle(atan2(yp, xp) - vR[i] - dRadBegin).get_positive();
//        double rad = CAngle(FastMath::table_atan2(yp, xp) - vR[i] - dRadBegin).get_positive();
        double rad = FastMath::table_atan2(yp, xp) - vR[i] - dRadBegin;
        while (rad >= 2*M_PI) rad -= 2*M_PI;
        while (rad < 0)       rad += 2*M_PI;
        int nPos = (int)round((rad/dRadReso));
        if (nPos == nP360) nPos = 0; //360回ったとき
        nPosMax = nPos; 
      }

      double xp = dx - vX[i];
      double yp = dy - vY[i];
      double dist = xp*xp+yp*yp;

      ccout->SetColor(ColorCout::eRed);

      /*
      if (nPosMin >= nLaserNum) {
        continue;
      }
      */
       
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
        if (dL != 0) {
//        if (dL == 0) dL = 50*1000;
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
  }
}

#if 0
void CSlamMap::SetLRFData2(const std::vector<const unsigned short *> &rvpLRFData, const std::vector<CCoordinates2D> &rvCoords, int nLaserNum, double dDegBegin, double dReso) {

  cout << "set lrf" << endl;

  mmtimer mt;

  auto *pCell = _pGrid->GetObjects();
  auto *pCellEnd = _pGrid->GetObjects()+_pGrid->GetTotalUnitNum();

  double dRadBegin = Deg2Rad(dDegBegin);
  double dRadReso = Deg2Rad(dReso);

  int nP360 = (int)(360/dReso);
  double dAlpha = sqrt(2.0)*_pGrid->GetXGridSize()/2;

  if (rvpLRFData.size() != rvCoords.size()) {
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

    for (size_t i=0; i<rvpLRFData.size(); ++i) {
      const unsigned short* pData = rvpLRFData[i];
      
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
//        double rad = CAngle(atan2(yp, xp) - vR[i] - dRadBegin).get_positive();
//        double rad = CAngle(FastMath::table_atan2(yp, xp) - vR[i] - dRadBegin).get_positive();
        double rad = FastMath::table_atan2(yp, xp) - vR[i] - dRadBegin;
        while (rad >= 2*M_PI) rad -= 2*M_PI;
        while (rad < 0)       rad += 2*M_PI;

        int nPos = (int)round((rad/dRadReso));
        if (nPos == nP360) nPos = 0; //360回ったとき
        nPosMin = nPos; 
      }
      {
        double xp = dxc[n2] - vX[i];
        double yp = dyc[n2] - vY[i];
//        double rad = CAngle(atan2(yp, xp) - vR[i] - dRadBegin).get_positive();
//        double rad = CAngle(FastMath::table_atan2(yp, xp) - vR[i] - dRadBegin).get_positive();
        double rad = FastMath::table_atan2(yp, xp) - vR[i] - dRadBegin;
        while (rad >= 2*M_PI) rad -= 2*M_PI;
        while (rad < 0)       rad += 2*M_PI;
        int nPos = (int)round((rad/dRadReso));
        if (nPos == nP360) nPos = 0; //360回ったとき
        nPosMax = nPos; 
      }

      double xp = dx - vX[i];
      double yp = dy - vY[i];
      double dist = xp*xp+yp*yp;

      ccout->SetColor(ColorCout::eRed);

      /*
      if (nPosMin >= nLaserNum) {
        continue;
      }
      */
       
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
        if (dL != 0) {
//        if (dL == 0) dL = 50*1000;
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
  }
}
#endif


//LRFの距離データと、LRFの座標(ワールド座標)を渡す
void CSlamMap::SetLRFData(const unsigned short *pLRFData, int nLaserNum, double dDegBegin, double dReso, const CCoordinates2D* pLRFCo) {

  int nDistNum = (int)(360/dReso);
  double *aDist = new double[nDistNum];
  for (int i=0; i<nDistNum; ++i) aDist[i]=-1;
  BoostVec temp(2);

  //物体のあるところに投票
  CAngle dCurDeg(dDegBegin, false);
  double dRX = _RobotCo.GetPos()(0);
  double dRY = _RobotCo.GetPos()(1);
  double dX,dY;
  double dReso1 = Deg2Rad(dReso);

  for (int i=0; i<nLaserNum; ++i, dCurDeg+=dReso1) {
    unsigned short nData = pLRFData[i];
    if (nData < 20) nData = USHRT_MAX;
    if (nData > 0) {
      int nPos = (int)round(dCurDeg.get_deg_positive()/dReso);
      if (nPos == nDistNum) nPos = 0;
      aDist[nPos] = nData;
      temp(0) = nData*cos(dCurDeg);
      temp(1) = nData*sin(dCurDeg);
      pLRFCo->TransformVector(temp);
      dX = temp(0); dY = temp(1);
      if ( ((dX-dRX)*(dX-dRX)+(dY-dRY)*(dY-dRY))  > _dRobotSize*_dRobotSize) {
        auto *pGrid = _pGrid->GetOneObjectInCarteCoords(dX, dY);
        if (pGrid) VoteWithBayesian(*pGrid, _dPOELRF, _dPOENLRF, true);
      }
    }
  }

  //物体のないところに投票
  double dLRFX = pLRFCo->GetPos()(0);
  double dLRFY = pLRFCo->GetPos()(1);
  double dLRFR = pLRFCo->GetRotation();
  for (int x=0; x<_nXGridNum; ++x) {
    for (int y=0; y<_nYGridNum; ++y) {
      auto *pGrid = _pGrid->GetOneObject(x,y);
      if (!pGrid->bChanged) {
        double xp = _pGrid->GridToPosX(x) - dLRFX;
        double yp = _pGrid->GridToPosY(y) - dLRFY;
        double dist = xp*xp+yp*yp;
        double deg = (Rad2Deg(atan2(yp, xp) - dLRFR));
//        double deg = (Rad2Deg(FastMath::table_atan2(yp, xp) - dLRFR));
        if (deg < 0) deg+=360; if (deg >= 360) deg-=360;
        int nDeg = (int)round(deg/dReso);
        if (nDeg == nDistNum) nDeg = 0;
        if (aDist[nDeg]*aDist[nDeg] > dist) {
          VoteWithBayesian(*pGrid, 1-_dPOELRF, 1-_dPOENLRF, true);
        }
      }
    }
  }

  delete[] aDist;
}
#if 0
//LRFの距離データと、LRFの座標(ワールド座標)を渡す
void CSlamMap::SetLRFData(const unsigned short *pLRFData, int nLaserNum, double dDegBegin, double dReso, const CCoordinates2D* pLRFCo) {

//  dout << "robotpos: " << _RobotCo << endl;
  int nDistNum = (int)(360/dReso);
  double *aDist = new double[nDistNum];
  for (int i=0; i<nDistNum; ++i) aDist[i]=-1;
  //物体のあるところに投票
//  double dCurDeg = 90-dDegBegin;
  double dCurDeg = dDegBegin;
  if (dCurDeg < 0) dCurDeg+=360;
  if (dCurDeg >= 360) dCurDeg-=360;
  Vec temp(2);
  for (int i=0; i<nLaserNum; ++i, dCurDeg+=dReso) {
    unsigned short nData = pLRFData[i];
    //temp 0827 物体がないところは無限遠とみなす
//    if (nData == 0) {
    if (nData < 20) {
      nData = USHRT_MAX;
    }
    if (nData > 0) {
      double d = dCurDeg;
      if (d < 0) d+=360;
      if (d >= 360) d-=360;
      int nPos = (int)round(d/dReso);
      if (nPos == nDistNum) nPos = 0;
      aDist[nPos] = nData;
      temp(0) = nData*cos(Deg2Rad(dCurDeg));
      temp(1) = nData*sin(Deg2Rad(dCurDeg));
      pLRFCo->TransformVector(temp);
      if ( hypot(temp(0) - _RobotCo.GetPos()(0), temp(1) - _RobotCo.GetPos()(1)) > _dRobotSize) {
        auto *pGrid = _pGrid->GetOneObjectInCarteCoords(temp(0), temp(1));
        if (pGrid) VoteWithBayesian(*pGrid, _dPOELRF, _dPOENLRF, true);
        //_pGrid->ApplyCarteCoords(temp, boost::bind(&VoteWithBayesian<SUnit>, _1, _dPOELRF, _dPOENLRF, true));
      }
    }
  }
  double dLRFR = pLRFCo->GetRotation();

  //物体のないところに投票
  for (int x=0; x<_nXGridNum; ++x) {
    for (int y=0; y<_nYGridNum; ++y) {
      double xp = _pGrid->GridToPosX(x) - pLRFCo->GetPos()(0);
      double yp = _pGrid->GridToPosY(y) - pLRFCo->GetPos()(1);
      double dist = sqrt(xp*xp+yp*yp);
      double deg = (Rad2Deg(atan2(yp, xp) - dLRFR));
      if (deg < 0) deg+=360; if (deg >= 360) deg-=360;
      int nDeg = (int)round(deg/dReso);
      if (nDeg == nDistNum) nDeg = 0;
      if (aDist[nDeg] > dist) {
        auto *pGrid = _pGrid->GetOneObject(x,y);
        VoteWithBayesian(*pGrid, 1-_dPOELRF, 1-_dPOENLRF, true);
        //_pGrid->ApplyGridCoords(x, y, boost::bind(&VoteWithBayesian<SUnit>, _1, 1-_dPOELRF, 1-_dPOENLRF, true));
      }

    }
  }
  delete[] aDist;
}
#endif


void CSlamMap::RegisterVoxel(float* p3DSingleData, unsigned char* pRGBSingleData, const CCoordinates3D &rCamCo) {
  if (temp.size() != 3) temp.resize(3);

  if (p3DSingleData[2] > _dVisionMaxLen) return;
    temp(0) = p3DSingleData[0];
    temp(1) = p3DSingleData[1];
    temp(2) = p3DSingleData[2];

    rCamCo.TransformVector(temp);
    boost::function1<void, SUnit&> f1 = 
      boost::bind(&VoteRGBWithBayesian, _1, _dPOEVision, _dPOENVision, pRGBSingleData,false);
    _pVoxel->ApplyCarteCoords(temp, f1);
}

void ClearSimpleUnit(SSimpleUnit &rUnit) {
  rUnit.nLabel = -1;
  rUnit.nVotedNum = 0;
}

bool IsSameGroupVision(SSimpleUnit& rFrom, SSimpleUnit& rTo, double dDistThr) {
  if ((abs(rFrom.nVotedNum-rTo.nVotedNum)) < dDistThr) return true;
  else return false;
}

//pPos, pRotはカメラのワールド座標
void CSlamMap::SetVisionData(float* p3DData, unsigned char* pRGBData, float* pPos, float* pRot) {

  mmtimer mt;
  //ビジョンのノイズ除去にグリッドを用いる
  _pGridForVision->ApplyAll(&ClearSimpleUnit);
  for (int i=0; i<_pGridForVision->GetXGridNum(); ++i) {
    for (int j=0; j<_pGridForVision->GetYGridNum(); ++j) {
      double z = (*(p3DData+(i+j*320)*3+2));
      int n=0;
      if (z>0) { //視差に直して登録
        n=(int)(round(_dBF/z));
      }
      _pGridForVision->ApplyGridCoords(i,j, boost::bind(&SimpleVote<SSimpleUnit>, _1, n));
    }
  }
  int nVisionLabelNum = _pGridForVision->Labeling(boost::bind(&IsSameGroupVision, _1, _2, _dVisionSameGroupDistThr));
  const int* anVisionLabelSize = _pGridForVision->GetLabelSize();

  SetArraytoCoords(_TempCo, pPos, pRot);
//  int nPointNum = 320*240;
  int xmin=_nCamXMin; int xmax=_nCamXMax;
  int ymin=_nCamYMin; int ymax=_nCamYMax;

  const Vec &rCamPos = _TempCo.GetPos();
  Vec _v1(3);
  Vec _v2(3);
  Vec _v3(3);
  Vec _v4(3);

  double dMax = _dBF/(_dVisionMaxLen+_pVoxel->GetXGridSize()*1.4142);
  _v1(0) = (xmin-_dXC)*_dB/dMax;
  _v1(1) = (ymin-_dYC)*_dB/dMax;
  _v1(2) = _dVisionMaxLen;
  _v2(0) = (xmax-_dXC)*_dB/dMax;
  _v2(1) = (ymax-_dYC)*_dB/dMax;
  _v2(2) = _dVisionMaxLen;
  _v3(0) = (xmax-_dXC)*_dB/dMax;
  _v3(1) = (ymin-_dYC)*_dB/dMax;
  _v3(2) = _dVisionMaxLen;
  _v4(0) = (xmin-_dXC)*_dB/dMax;
  _v4(1) = (ymax-_dYC)*_dB/dMax;
  _v4(2) = _dVisionMaxLen;

  _TempCo.TransformVector(_v1);
  _TempCo.TransformVector(_v2);
  _TempCo.TransformVector(_v3);
  _TempCo.TransformVector(_v4);

  double dxmin = min(_v1(0), min(_v2(0), min(_v3(0), min(_v4(0), rCamPos(0)))));
  double dymin = min(_v1(1), min(_v2(1), min(_v3(1), min(_v4(1), rCamPos(1)))));
  double dzmin = min(_v1(2), min(_v2(2), min(_v3(2), min(_v4(2), rCamPos(2)))));
  double dxmax = max(_v1(0), max(_v2(0), max(_v3(0), max(_v4(0), rCamPos(0)))));
  double dymax = max(_v1(1), max(_v2(1), max(_v3(1), max(_v4(1), rCamPos(1)))));
  double dzmax = max(_v1(2), max(_v2(2), max(_v3(2), max(_v4(2), rCamPos(2)))));

  int nxmin = _pVoxel->GetXPos(dxmin);
  int nymin = _pVoxel->GetYPos(dymin);
  int nzmin = _pVoxel->GetZPos(dzmin);
  int nxmax = _pVoxel->GetXPos(dxmax);
  int nymax = _pVoxel->GetYPos(dymax);
  int nzmax = _pVoxel->GetZPos(dzmax);

//  int cnt = 0;
//  int cnt2 = 0;
//  unsigned char hoge[] = {255,255,255};
  Vec vPos(3);
  Vec vPosMin(3);
  Vec vPosMax(3);
  Vec vVecGrid(3); 
   //本当のmin-maxを計算するのは面倒なので
  vVecGrid(0) = _pVoxel->GetXGridSize()*1.7320508/2;
  vVecGrid(1) = _pVoxel->GetYGridSize()*1.7320508/2;
  vVecGrid(2) = 0;
  nzmin = max(2, nzmin); //いんちき
  const SSimpleUnit *pVisionUnit = _pGridForVision->GetObjects();

  for (int x=nxmin; x<=nxmax; ++x) {
    for (int y=nymin; y<=nymax; ++y) {
      for (int z=nzmin; z<=nzmax; ++z) {
        if (_pVoxel->IsInBound(x,y,z)) {
          vPos(0) = _pVoxel->GridToPosX(x);
          vPos(1) = _pVoxel->GridToPosY(y);
          vPos(2) = _pVoxel->GridToPosZ(z); //中心点
          _TempCo.InverseTransformVector(vPos);
          vPosMin = vPos-vVecGrid;
          vPosMax = vPos+vVecGrid;
          double d1 = _dBF/vPosMin(2);
          double d2 = _dBF/vPosMax(2);
          int xposmin = (int)round(vPosMin(0)/_dB*d1+_dXC);
          int yposmin = (int)round(vPosMin(1)/_dB*d1+_dYC); 
          int xposmax = (int)round(vPosMax(0)/_dB*d2+_dXC);
          int yposmax = (int)round(vPosMax(1)/_dB*d2+_dYC); //カメラ画像だとどこか？
          if ( (xposmax >= xmin) && (xposmin <= xmax) && (yposmax >= ymin) && (yposmin <= ymax) ) {
            float dVZ = 0;
            xposmin = max(xposmin, xmin);
            yposmin = max(yposmin, ymin);
            xposmax = min(xposmax, xmax);
            yposmax = min(yposmax, ymax);
            for (int xx = xposmin; xx < xposmax; ++xx) {
              for (int yy = yposmin; yy < yposmax; ++yy) {
                float dZ1 = *(p3DData+(xx+yy*320)*3+2);
                if (anVisionLabelSize[(pVisionUnit+_pGridForVision->GetArrayPos(xx,yy))->nLabel] <= _nVisionNoiseThr) {
                  dZ1 = -1;
                }
                dVZ = max(dVZ, dZ1);
              }
            }
            if ((vPos(2) < _dVisionMaxLen ) && ( (dVZ > (vPos(2))) || (dVZ == 0))){
              _pVoxel->ApplyGridCoords(x, y, z, boost::bind(&VoteWithBayesian<SUnit>, _1, 1-_dPOEVision, 1-_dPOENVision, false));
            }
          }
        }
      }
    }
  }

  float *pCur3DData = p3DData;
  unsigned char *pCurRGBData = pRGBData;
  for (int i=ymin; i<ymax; ++i) {
    for (int j=xmin; j<xmax; ++j) {
      pCur3DData = p3DData+(j+i*320)*3;
      if ((pCur3DData[2] > 0) && 
          (anVisionLabelSize[(pVisionUnit+_pGridForVision->GetArrayPos(j,i))->nLabel] > _nVisionNoiseThr))
      {
        pCurRGBData = pRGBData+(j+i*320)*3;
        RegisterVoxel(pCur3DData, pCurRGBData, _TempCo);
      }
    }
  }
}


void CSlamMap::SetInitRobotPos(double rx, double ry, double radious) {

  const Vec rPos = _RobotCo.GetPos();
//  int nRobotX = _pGrid->GetXPos(rx);
//  int nRobotY = _pGrid->GetYPos(ry);
  _dRobotSize = radious;
  dout << "_dRobotSize = " << radious << endl;

  /*
  double dSearchLen = 0.5 + radious/_pGrid->GetXGridSize();
  int nSearchXLenMax = (int)dSearchLen;
  int nXMax = min(nRobotX+nSearchXLenMax+1, _nXGridNum);
  int nXMin = max(nRobotX-nSearchXLenMax  , 0);
  */

}

void ForgetAndInitLabel(SUnit& rUnit, int nRegistEternalScore, int nForgetScore) {
  if ( rUnit.nVotedNum < nRegistEternalScore) {
    rUnit.nVotedNum = max( rUnit.nVotedNum-nForgetScore, 0);
  }
  rUnit.nLabel = -1;
}

void Forget(SUnit& rUnit, int nRegistEternalScore, int nForgetScore) {
  if ( rUnit.nVotedNum < nRegistEternalScore) {
    rUnit.nVotedNum = max( rUnit.nVotedNum-nForgetScore, 0);
  }
}

void InitLabel(SUnit& rUnit) {
  rUnit.nLabel = -1;
  rUnit.bChanged = false;
}


bool IsUnitValid(SUnit& rTarget, SUnit& rBefore, double dThr) {
  return ((rTarget.dLikelihood>dThr) && (rBefore.dLikelihood>dThr));
//  return (rTarget.dLikelihood>dThr);
}

bool DummyFunc(SUnit& rTarget1, SUnit& rTarget2) {return true;}

void Temp(SUnit& rTarget, int* pN1, int* pN2) {
  if (rTarget.nLabel==0) {
    rTarget.dRAverage = 255;
    rTarget.dBAverage = 0;
    rTarget.dGAverage = 0;
    ++(*pN1);
  }
  if (rTarget.nLabel==1) {
    rTarget.dRAverage = 128;
    rTarget.dBAverage = 128;
    rTarget.dGAverage = 128;
    ++(*pN2);
  }
}

void InitSearchLabel(SSearchMapState &State) {
  State.nLabel = -1;
}

void CSlamMap::ApplyAllGrid(const boost::function1<void, SUnit&>& f1) {
  _pGrid->ApplyAll(f1);
}

void CSlamMap::ApplyAllVoxel(const boost::function1<void, SUnit&>& f1) {
  _pVoxel->ApplyAll(f1);
}




//#define SHOW_DBG
void CSlamMap::UpdateState() {

  using namespace boost;
  mmtimer mt;

  //forget & init label 
  boost::function1<void, SUnit&> f1 = boost::bind(&InitLabel, _1);
  _pVoxel->ApplyAll(f1);
  _pGrid->ApplyAll(f1);

#ifdef SHOW_DBG
  dout << "  first " << mt.elapsed() << endl;
#endif

  int nLabelNum = _pVoxel->Labeling(boost::bind(&IsUnitValid, _1, _2, _dObstacleThrVision));
  const int* anLabelSize = _pVoxel->GetLabelSize();

/*  
  cout << "size: " << nLabelNum << endl;
  for(int i=0; i<nLabelNum; ++i) {
    cout << anLabelSize[i] << " ";
  }
  cout << endl;
*/

#ifdef SHOW_DBG
  dout << "  label " << mt.elapsed() << endl;
#endif

//  const SUnit* pVoxel = _pVoxel->GetObjects();
  const SUnit* pGrid  = _pGrid->GetObjects();

  //OBSTACLE/FREE/UNKNOWN
  for (int i=0; i<_nXGridNum; ++i) {
    for (int j=0; j<_nYGridNum; ++j) {
      const SUnit* pCurGrid = pGrid+_pGrid->GetArrayPos(i,j);
      //init label
      SSearchMapState* pState = _pSearchMap->GetOneObject(i,j);
      pState->nLabel = -1;
//      _v2SearchMap[i][j].nLabel = -1;
      //check obstacle
      if (pCurGrid->dLikelihood > _dObstacleThrLRF) {
        _pSearchMap->GetOneObject(i,j)->State = SSearchMapState::OBSTACLE_LRF;
//        _v2SearchMap[i][j].State = SSearchMapState::OBSTACLE_LRF;
      }
      else {
        double nMax = -1;
        for (int k=2; k<_nZGridNum; ++k) {
//          int nLabel = (rTemp+k)->nLabel;
          const SUnit* pCur = _pVoxel->GetObjectInGridCoords(i,j,k);
          int nLabel = pCur->nLabel;
          if ( (nLabel!=-1) && (anLabelSize[nLabel] > _nMinRegion)) {
            nMax = max(nMax, pCur->dLikelihood );
          }
        }
        if ( nMax > _dObstacleThrVision ) 
        {
          _pSearchMap->GetOneObject(i,j)->State = SSearchMapState::OBSTACLE_VISION;
  //        _v2SearchMap[i][j].State = SSearchMapState::OBSTACLE_VISION;
        }
        else if ( (_pVoxel->GetObjectInGridCoords(i,j,0)->dLikelihood > _dObstacleThrVision) ||
                  (_pVoxel->GetObjectInGridCoords(i,j,1)->dLikelihood > _dObstacleThrVision) ){
          _pSearchMap->GetOneObject(i,j)->State = SSearchMapState::FREE;
//          _v2SearchMap[i][j].State = SSearchMapState::FREE;
        }
        else {
          _pSearchMap->GetOneObject(i,j)->State = SSearchMapState::UNKNOWN;
//          _v2SearchMap[i][j].State = SSearchMapState::UNKNOWN;
        }
      }
    }
  }


#ifdef SHOW_DBG
  dout << "  setsearchmap " << mt.elapsed() << endl;
#endif
//  dout << _dRobotSize;

  double dRobotLen = 0.5 + _dRobotSize/_pGrid->GetXGridSize();

//  dout << " -> " << dRobotLen << endl;
  const Vec rPos = _RobotCo.GetPos();
  int nRobotX = _pGrid->GetXPos(rPos(0));
  int nRobotY = _pGrid->GetYPos(rPos(1));
  int nXMax = min(nRobotX+(int)(round(dRobotLen+1)), _nXGridNum);
  int nXMin = max(nRobotX-(int)(round(dRobotLen))  , 0);

  double dSearchLen = dRobotLen*2;
  //NEAR_OBSTACLE
  for (int i=0; i<_nXGridNum; ++i) {
    for (int j=0; j<_nYGridNum; ++j) {
      SSearchMapState *pCurState = _pSearchMap->GetOneObject(i,j);
      pCurState->dDistanceFromObs = DBL_MAX;
//      _v2SearchMap[i][j].dDistanceFromObs = DBL_MAX;
//      if (!_v2SearchMap[i][j].IsOccupied()) {
      if (!pCurState->IsOccupied()) {
        int nSearchXLenMax = (int)dSearchLen;
        int nXMax = min(i+nSearchXLenMax+1, _nXGridNum);
        int nXMin = max(i-nSearchXLenMax  , 0);
        for (int x = nXMin; x < nXMax; ++x) {
          int nSearchYLen = (int)floor(sqrt(dSearchLen*dSearchLen-(x-i)*(x-i)));
          int nYMax = min(j+nSearchYLen+1, _nYGridNum);
          int nYMin = max(j-nSearchYLen  , 0);
          for (int y = nYMin; y < nYMax; ++y) {
            SSearchMapState *pTargetState = _pSearchMap->GetOneObject(x,y);
//            if (_v2SearchMap[x][y].IsOccupied() && (_v2SearchMap[x][y].State != SSearchMapState::NEAR_OBSTACLE)) {
            if (pTargetState->IsOccupied() && (pTargetState->State != SSearchMapState::NEAR_OBSTACLE)) {
              double d = sqrt(double((x-i)*(x-i)+(y-j)*(y-j)));
              if (d < dRobotLen) {
                pCurState->State = SSearchMapState::NEAR_OBSTACLE;
              }
              else {
                pCurState->dDistanceFromObs = min(pCurState->dDistanceFromObs,d);
//                pCurState->dDistanceFromObs = min(pCurState->dDistanceFromObs,d);
              }
            }
          }
        }
      }
    }
  }


  /*
  int nCurLabel = 0;
  int anLabelSize2[1000];
  SSearchMapState::state aeLabelState[1000];
  _pSearchMap->ApplyAll(InitSearchLabel);
  for (int i=0; i<_nXGridNum; ++i) {
    for (int j=0; j<_nYGridNum; ++j) {
      _v2SearchMap[i][j].nLabel = -1;
    }
  }
  //labeling grid
  for (int i=0; i<_nXGridNum; ++i) {
    for (int j=0; j<_nYGridNum; ++j) {
      if (_v2SearchMap[i][j].nLabel == -1) {
        anLabelSize2[nCurLabel] = SetLabelGrid(nCurLabel, i, j ,_v2SearchMap[i][j].State);
        aeLabelState[nCurLabel] = _v2SearchMap[i][j].State;
        ++nCurLabel;
      }
    }
  }
  */
  

  int nRobotXLenMax = (int)(dRobotLen);
  //ロボットを登録
  nXMax = min(nRobotX+nRobotXLenMax+1, _nXGridNum);
  nXMin = max(nRobotX-nRobotXLenMax  , 0);
  for (int x = nXMin; x < nXMax; ++x) {
    int nSearchYLen = (int)floor(sqrt(dRobotLen*dRobotLen-(x-nRobotX)*(x-nRobotX)));
    int nYMax = min(nRobotY+nSearchYLen+1, _nYGridNum);
    int nYMin = max(nRobotY-nSearchYLen  , 0);
    for (int y = nYMin; y < nYMax; ++y) {
      _pSearchMap->GetOneObject(x,y)->State = SSearchMapState::ROBOT;
//      _v2SearchMap[x][y].State = SSearchMapState::ROBOT;
    }
  }

}

//そのラベルに属するグリッドの数を返す
int CSlamMap::SetLabelGrid(int nLabelNo, int x, int y, SSearchMapState::state eBeforeState) {

  /*
  if ( (x<0) || (y<0)
       || (x>=_pGrid->GetXGridNum()) || (y>=_pGrid->GetYGridNum()) 
       || (_v2SearchMap[x][y].nLabel != -1) 
       || (_v2SearchMap[x][y].State != eBeforeState) 
       )
  {
    return 0;
  }
  else {

  int label = _v2SearchMap[x][y].nLabel;
  if (label != -1) {
    return 0;
  }
  if (_v2SearchMap[x][y].State != eBeforeState) {
    return 0;
  }

    SSearchMapState::state eCurState = _v2SearchMap[x][y].State;
    _v2SearchMap[x][y].nLabel = nLabelNo;
    int nScore = 1 + SetLabelGrid(nLabelNo, x-1, y, eCurState) 
                   + SetLabelGrid(nLabelNo, x, y-1, eCurState)
                   + SetLabelGrid(nLabelNo, x+1, y, eCurState)
                   + SetLabelGrid(nLabelNo, x, y+1, eCurState) ;
    return nScore;
  }
  */
  return 0;
}

void CSlamMap::DumpSearchMap(float *pMap) {

  ofstream fout("res.dat");

//  fout << "#2f(" ;

  for (int i=0; i<_nYGridNum; ++i) {
//    fout << "(";
    for (int j=0; j<_nXGridNum; ++j) {
//      fout << _v2SearchMap[j][i].IsOccupied() << " ";
      fout << _pSearchMap->GetOneObject(j,i)->IsOccupied() << " ";
    }
//    fout << ")" << endl;
    fout << endl;
  }
  fout << endl;
//  fout << ")" << endl;
}

bool CSlamMap::ObstacleOnPath(const std::vector<location>* pPath) const {
  if (pPath->size()<2) return true;

  for (size_t i=0; i<(pPath->size()-1); ++i) {
    if (CheckObstacleGrid( (*pPath)[i].x,(*pPath)[i].y,(*pPath)[i+1].x,(*pPath)[i+1].y) ) {
      /*
      cout << "SlamMap.cpp ObstacleOnPath=true:" << " i= " << (int)i;
      cout << " (" << (*pPath)[i].x << " " << (*pPath)[i].y << ")->(" << (*pPath)[i+1].x << " " << (*pPath)[i+1].y << ")" << endl;
      dout << "SlamMap.cpp ObstacleOnPath=true:" << " i= " << (int)i;
      dout << " (" << (*pPath)[i].x << " " << (*pPath)[i].y << ")->(" << (*pPath)[i+1].x << " " << (*pPath)[i+1].y << ")" << endl;
      */
      return true;
    }
  }
  return false;
}



