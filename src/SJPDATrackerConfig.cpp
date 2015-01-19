#include "StdAfx_MOTracking.h"
#include "SJPDATrackerConfig.h"
#include "ColorCout.h"
#include <boost/filesystem.hpp>
using namespace std;

void SJPDATrackerConfig::LoadFromIni(const std::string &sIniFileName2) {

  STrackerConfig::LoadFromIni(sIniFileName2);

  string sModuleName = "PT";
  if (!boost::filesystem::exists(sIniFileName2)) {
    ESStreamException ess; ess << sIniFileName2 << " not exist";
    throw ess;
  }
  boost::filesystem::path logfile(sIniFileName2);
  string sIniFileName = boost::filesystem::absolute(logfile).string();
  const int nBufSize = 2048;
  char sBuf[nBufSize];

  _dViewerSizeY = GetPrivateProfileDouble(sModuleName.c_str(), "ViewerSizeY", 800, sIniFileName.c_str());
  _dViewerRange = GetPrivateProfileDouble(sModuleName.c_str(), "ViewerRange", 20000, sIniFileName.c_str());
  _nMaxUpdateGridFrame = GetPrivateProfileInt(sModuleName.c_str(), "MaxUpdateFrame", INT_MAX, sIniFileName.c_str());
  _bUpdateGrid = (GetPrivateProfileInt(sModuleName.c_str(), "UpdateGrid", 1, sIniFileName.c_str()) != 0);

  _dLRFObsOdds = GetPrivateProfileDouble(
    sModuleName.c_str(), "ObsOdds", 1.0, sIniFileName.c_str());
  _dLRFFreeOdds = GetPrivateProfileDouble(
    sModuleName.c_str(), "FreeOdds", 0.1, sIniFileName.c_str());
  _dLRFOddsMax = GetPrivateProfileDouble(
    sModuleName.c_str(), "OddsMax", 10.0, sIniFileName.c_str());
  _dLRFOddsMin = GetPrivateProfileDouble(
    sModuleName.c_str(), "OddsMin", -10.0, sIniFileName.c_str());
  double d1  = GetPrivateProfileDouble(
    sModuleName.c_str(), "ObsThr", 0.99, sIniFileName.c_str());
  _dObsThr = log(d1/(1-d1));
  double d2 = GetPrivateProfileDouble(
    sModuleName.c_str(), "FreeThr", 0.01, sIniFileName.c_str());
  _dFreeThr = log(d2/(1-d2));

  for (size_t i=0; i<_vLRFCos.size(); ++i) {
    GetPrivateProfileString(sModuleName.c_str(), (string("SaveGridFileName")+boost::lexical_cast<string>(i)).c_str(), "", sBuf, nBufSize, sIniFileName.c_str());
    _vsSaveGridFileNames.push_back(sBuf);
    GetPrivateProfileString(sModuleName.c_str(), (string("LoadGridFileName")+boost::lexical_cast<string>(i)).c_str(), "", sBuf, nBufSize, sIniFileName.c_str());
    _vsLoadGridFileNames.push_back(sBuf);
  }

  _bDrawGrid = (GetPrivateProfileInt(sModuleName.c_str(), "DrawGrid", 0, sIniFileName.c_str()) != 0);
  _bDrawViewer = (GetPrivateProfileInt(sModuleName.c_str(), "DrawViewer", 1, sIniFileName.c_str()) != 0);
  _nScreenWidth = GetPrivateProfileInt(sModuleName.c_str(), "ScreenWidth", 1024, sIniFileName.c_str());
  _bShowHypoLines = (GetPrivateProfileInt(sModuleName.c_str(), "ShowHypoLines", 1, sIniFileName.c_str()) != 0);
  _dGridSize = GetPrivateProfileDouble(sModuleName.c_str(), "GridSize", 100.0, sIniFileName.c_str());
  _dPolarGridMaxLen = GetPrivateProfileDouble(sModuleName.c_str(), "PolarGridMaxLen", 20*1000, sIniFileName.c_str());
  _bChaseLRF = (GetPrivateProfileInt(sModuleName.c_str(), "ChaseLRF", 0, sIniFileName.c_str()) != 0);
  GetPrivateProfileString(sModuleName.c_str(), "FeatureDir", _sFeatureDir.c_str(), sBuf, nBufSize, sIniFileName.c_str());
  _sFeatureDir = sBuf;
  _nIniterType = GetPrivateProfileInt(sModuleName.c_str(), "IniterType", -1, sIniFileName.c_str());
  if (_nIniterType < 0) {
    bool bUsePolarInitializer = (GetPrivateProfileInt(sModuleName.c_str(), "UsePolarInitializer", 1, sIniFileName.c_str()) != 0);
    if (bUsePolarInitializer) {
      ccout << "using polar" << endl;
      _nIniterType = 1;
    }
    else {
      ccout->SetColor(ColorCout::eYellow);
      ccout << "initer type not set" << endl;
      ccout << "using grid" << endl;
      _nIniterType = 0;
    }
  }
  else {
    ccout << "IniterType: " << _nIniterType << endl;
    if (_nIniterType == 1) {
      ccout << " using polar" << endl;
    }
    else if (_nIniterType == 0) {
      ccout << " using grid" << endl;
    }
    else if (_nIniterType == 2) {
      ccout << " using passthrough" << endl;
    }
    else {
      ccout << " warning: unknown grid!" << endl;
    }
  }

  _bWriteEventLog = (GetPrivateProfileInt(sModuleName.c_str(), "WriteEventLog", 0, sIniFileName.c_str()) != 0);

  _dRemoveIntervalDistThrMax = GetPrivateProfileDouble(
    sModuleName.c_str(), "RemoveIntervalDistThrMax", 2000, sIniFileName.c_str());
  _dRemoveIntervalDistThrMin = GetPrivateProfileDouble(
    sModuleName.c_str(), "RemoveIntervalDistThrMin", 500, sIniFileName.c_str());
  _dRemoveIntervalDistMulti = GetPrivateProfileDouble(
    sModuleName.c_str(), "RemoveIntervalDistMulti", 0.1, sIniFileName.c_str());
  _dClusterLenThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "ClusterLenThr", 300.0, sIniFileName.c_str());
  _dClusterAngleThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "ClusterAngleThr", 2.5, sIniFileName.c_str());
  _dNewObjPointExtractThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "NewObjPointExtractThr", 0.1, sIniFileName.c_str());

  _dInitTotalLenThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "InitTotalLenThr", 150, sIniFileName.c_str());
  _dInitWidthTotalThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "InitWidthTotalThr", 200, sIniFileName.c_str());
  _dInitDistBeteenEdgeThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "InitDistBeteenEdgeThr", 180, sIniFileName.c_str());
  _dInitMinDistFromOtherThrPerson = GetPrivateProfileDouble(
    sModuleName.c_str(), "InitMinDistFromOtherThrPerson", 700, sIniFileName.c_str());
  _dInitDistanceThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "InitDistanceThr", 0, sIniFileName.c_str());

  _bDumpDebugInfo = (GetPrivateProfileInt(sModuleName.c_str(), "DumpDebugInfo", 0, sIniFileName.c_str()) != 0);
  _bShowDebugString = (GetPrivateProfileInt(sModuleName.c_str(), "ShowDebugString", 0, sIniFileName.c_str()) != 0);
  _bUseDynamicProgramming = (GetPrivateProfileInt(sModuleName.c_str(), "UseDynamicProgramming", 1, sIniFileName.c_str()) != 0);
  _bUseJIPDA = (GetPrivateProfileInt(sModuleName.c_str(), "UseJIPDA", 1, sIniFileName.c_str()) != 0);
  _bPerformCarTrack = (GetPrivateProfileInt(sModuleName.c_str(), "PerformCarTrack", 0, sIniFileName.c_str()) != 0);
  _bSetPersonSplittable = (GetPrivateProfileInt(sModuleName.c_str(), "SetPersonSplittable", 0, sIniFileName.c_str()) != 0); 
  _bUseRemovedHypoInWeighting = (GetPrivateProfileInt(sModuleName.c_str(), "UseRemovedHypoInWeighting", 0, sIniFileName.c_str()) != 0); 

  _dRemoveExistanceRateThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "RemoveExistanceRateThr", 0.001, sIniFileName.c_str());
  _dRemoveDisparsionThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "RemoveDisparsionThr", 800, sIniFileName.c_str());
  _dIntervalStandardLen = GetPrivateProfileDouble(
    sModuleName.c_str(), "IntervalStandardLen", 15000, sIniFileName.c_str());

  //0 means default value
  _dPersonR1Ave = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonR1Ave", 0, sIniFileName.c_str());
  _dPersonR1StdDev = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonR1StdDev", 0, sIniFileName.c_str());
  _dPersonR1Max = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonR1Max", 0, sIniFileName.c_str());
  _dPersonR1Min = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonR1Min", 0, sIniFileName.c_str());
  _dPersonR2Ave = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonR2Ave", 0, sIniFileName.c_str());
  _dPersonR2StdDev = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonR2StdDev", 0, sIniFileName.c_str());
  _dPersonR2Max = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonR2Max", 0, sIniFileName.c_str());
  _dPersonR2Min = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonR2Min", 0, sIniFileName.c_str());

  _dPersonXN = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonXN", 0, sIniFileName.c_str());
  _dPersonRadN = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonRadN", 0, sIniFileName.c_str());
  _dPersonVN1 = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonVN1", 0, sIniFileName.c_str());
  _dPersonRVN1 = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonRVN1", 0, sIniFileName.c_str());
  _dPersonVN2 = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonVN2", 0, sIniFileName.c_str());
  _dPersonRVN2 = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonRVN2", 0, sIniFileName.c_str());
  _dPersonLowVelThr = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonLowVelThr", 0, sIniFileName.c_str());
  _dPersonRVNLowVel = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonRVNLowVel", 0, sIniFileName.c_str());

  _dPersonSigma = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonSigma", 0, sIniFileName.c_str());
  _dPersonOtherMOLenAve = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonOtherMOLenAve", 0, sIniFileName.c_str());
  _dPersonOtherMOLenStdDev = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonOtherMOLenStdDev", 0, sIniFileName.c_str());
  _dPersonFPExistLen = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonFPExistLen", 0, sIniFileName.c_str());
  _dPersonFPNearProb = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonFPNearProb", 0, sIniFileName.c_str());
  _dPersonPassProbTotal = GetPrivateProfileDouble(
    sModuleName.c_str(), "PersonPassProbTotal", 0, sIniFileName.c_str());


  _bDumpSVMData = (GetPrivateProfileInt(sModuleName.c_str(), "DumpSVMData", 0, sIniFileName.c_str()) != 0);

  _vDetectRegion.clear();
  for (int i=0; i<(int)_vLRFCos.size(); ++i) {
    string sName = string("DetectRegion")+boost::lexical_cast<string>(i);
    GetPrivateProfileString(sModuleName.c_str(), sName.c_str(), "", sBuf, 512, sIniFileName.c_str());
    if (strlen(sBuf) != 0) {
      char* p=sBuf;
      while (*p) {
        if (*p==',') *p = ' ';
        ++p;
      }
      istringstream iss(sBuf);
      int nSize;
      iss >> nSize;
      cout << sName << ":";
      CPolygonalRegion Region;
      for (int i=0; i<nSize; ++i) {
        double x, y;
        iss >> x >> y;
        Region.AddPoint(x,y);
        cout << "(" << x << "," << y << ") ";
      }
      cout << endl;
      _vDetectRegion.push_back(Region);
    }
  }
}