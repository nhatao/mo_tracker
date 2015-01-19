#include "StdAfx_MOTracking.h"
#include "TrackerConfig.h"
#include "ColorCout.h"

#include <boost/filesystem.hpp>

using namespace std;

  STrackerConfig(const std::string &sName="");
  virtual void ReadFromIniFile(const std::string &rsFileName);


void STrackerConfig::LoadFromIni(const std::string &sIniFileName2) {

  if (!boost::filesystem::exists(sIniFileName2)) {
    ESStreamException ess; ess << sIniFileName2 << " not exist";
    throw ess;
  }
  boost::filesystem::path logfile(sIniFileName2);
  string sIniFileName = boost::filesystem::absolute(logfile).string();

  _bAuto = true;
  const int nBufSize = 2048;
  char sBuf[nBufSize];
  string sModuleName = "PT";
  _nMode = GetPrivateProfileInt(sModuleName.c_str(), "Mode", -1, sIniFileName.c_str());
  int nLRFNum = GetPrivateProfileInt(sModuleName.c_str(), "LRFNum", 4, sIniFileName.c_str());
  if (nLRFNum <= 0) {
    throw std::logic_error("LRFNum=0");
  }
  if ( (_nMode < 0) || (_nMode > 3) ) {
    throw std::logic_error("Invalid Mode");
  }

  for (int i=0; i<nLRFNum; ++i) {
    double dX = GetPrivateProfileDouble(sModuleName.c_str(), (string("LRFX")+boost::lexical_cast<string>(i)).c_str(), 0, sIniFileName.c_str());
    double dY = GetPrivateProfileDouble(sModuleName.c_str(), (string("LRFY")+boost::lexical_cast<string>(i)).c_str(), 0, sIniFileName.c_str());
    double dR = GetPrivateProfileDouble(sModuleName.c_str(), (string("LRFR")+boost::lexical_cast<string>(i)).c_str(), 0, sIniFileName.c_str());
    GetPrivateProfileString(sModuleName.c_str(), (string("SaveGridFileName")+boost::lexical_cast<string>(i)).c_str(), "", sBuf, nBufSize, sIniFileName.c_str());
    _vsSaveGridFileNames.push_back(sBuf);
    GetPrivateProfileString(sModuleName.c_str(), (string("LoadGridFileName")+boost::lexical_cast<string>(i)).c_str(), "", sBuf, nBufSize, sIniFileName.c_str());
    _vsLoadGridFileNames.push_back(sBuf);
    
    CCoordinates2D co(dX, dY, Deg2Rad(dR));
    _vLRFCos.push_back(co);

    if (_nMode == 0) { //file
      GetPrivateProfileString(sModuleName.c_str(), (string("LRFLog")+boost::lexical_cast<string>(i)).c_str(), "0", sBuf, nBufSize, sIniFileName.c_str());
      _vsFileName.push_back(sBuf);
      _bAuto = (GetPrivateProfileInt(sModuleName.c_str(), "AutoStart", 1, sIniFileName.c_str()) != 0);

      SLRFProperty s;
      s._nElemNum = GetPrivateProfileInt(sModuleName.c_str(), 
        (string("PointNum")+boost::lexical_cast<string>(i)).c_str(), 
        1081, sIniFileName.c_str());
      s._dReso = GetPrivateProfileDouble(sModuleName.c_str(), 
        (string("Resolution")+boost::lexical_cast<string>(i)).c_str(), 
        Deg2Rad(0.25), sIniFileName.c_str());
      s._dFirstAngle = GetPrivateProfileDouble(sModuleName.c_str(), 
        (string("FirstAngle")+boost::lexical_cast<string>(i)).c_str(), 
        Deg2Rad(-135), sIniFileName.c_str());
      s._dMinRange = GetPrivateProfileDouble(sModuleName.c_str(), 
        (string("MinRange")+boost::lexical_cast<string>(i)).c_str(), 
        20, sIniFileName.c_str());
      s._dMaxRange = GetPrivateProfileDouble(sModuleName.c_str(), 
        (string("MaxRange")+boost::lexical_cast<string>(i)).c_str(), 
        30*1000, sIniFileName.c_str());
      s._dScanTime = GetPrivateProfileDouble(sModuleName.c_str(), 
        (string("ScanTime")+boost::lexical_cast<string>(i)).c_str(), 
        0.025, sIniFileName.c_str());
      s._dTimeIncrement = GetPrivateProfileDouble(sModuleName.c_str(), 
        (string("TimeIncrement")+boost::lexical_cast<string>(i)).c_str(), 
        0.025/(360/0.25), sIniFileName.c_str());
      _vLogLRFProperty.push_back(s);
    }
    else if (_nMode == 1) { //URG
      int nPort = GetPrivateProfileInt(sModuleName.c_str(), (string("LRFPort")+boost::lexical_cast<string>(i)).c_str(), 0, sIniFileName.c_str());
      _vnPorts.push_back(nPort);
    }
    else if (_nMode == 2) { //SICK
      int nPort = GetPrivateProfileInt(sModuleName.c_str(), (string("LRFAddrPort")+boost::lexical_cast<string>(i)).c_str(), 0, sIniFileName.c_str());
      _vnPorts.push_back(nPort);
      GetPrivateProfileString(sModuleName.c_str(), (string("LRFAddr")+boost::lexical_cast<string>(i)).c_str(), "0", sBuf, nBufSize, sIniFileName.c_str());
      _vsAddrName.push_back(sBuf);
      int nFogMode = GetPrivateProfileInt(sModuleName.c_str(), (string("LRFFogMode")+boost::lexical_cast<string>(i)).c_str(), 0, sIniFileName.c_str());
      int nLRFNPalseMode = GetPrivateProfileInt(sModuleName.c_str(), (string("LRFNPalseMode")+boost::lexical_cast<string>(i)).c_str(), 0, sIniFileName.c_str());
      _vnFogMode.push_back(nFogMode);
      _vnNPalse.push_back(nLRFNPalseMode);
      
    }
    else if (_nMode == 3) {
      _bWaitForTransform = (GetPrivateProfileInt(sModuleName.c_str(), "WaitForTransform", 1, sIniFileName.c_str()) != 0);
    }
    else {
      throw std::logic_error("invalid mode");
    }
  }
  _dViewerSizeY = GetPrivateProfileDouble(sModuleName.c_str(), "ViewerSizeY", 800, sIniFileName.c_str());
  _dViewerRange = GetPrivateProfileDouble(sModuleName.c_str(), "ViewerRange", 20000, sIniFileName.c_str());

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

  _dFrameUpdateMinTime = GetPrivateProfileDouble(
    sModuleName.c_str(), "FrameUpdateMinTime", 0.095, sIniFileName.c_str());
  
  _nMaxUpdateGridFrame = GetPrivateProfileInt(sModuleName.c_str(), "MaxUpdateFrame", INT_MAX, sIniFileName.c_str());
  _bUpdateGrid = (GetPrivateProfileInt(sModuleName.c_str(), "UpdateGrid", 1, sIniFileName.c_str()) != 0);

  if (_nMode == 0) {
    GetPrivateProfileString(sModuleName.c_str(), "OdmFileName", "", sBuf, nBufSize, sIniFileName.c_str());
    _sOdmFileName = sBuf;
  }

  _nMaxTargetNum = GetPrivateProfileInt(sModuleName.c_str(), "MaxTargetNum", 30, sIniFileName.c_str()); 

  _bShowHypoLines = (GetPrivateProfileInt(sModuleName.c_str(), "ShowHypoLines", 1, sIniFileName.c_str()) != 0);



  _dStartTime = GetPrivateProfileDouble(sModuleName.c_str(), "Log_FirstTime", 0.0, sIniFileName.c_str());
  _dEndTime = GetPrivateProfileDouble(sModuleName.c_str(), "Log_EndTime", DBL_MAX, sIniFileName.c_str());
  _dProcResetTime = GetPrivateProfileDouble(sModuleName.c_str(), "ProcResetTime", -1, sIniFileName.c_str());
  cout << "ResetTime: " << _dProcResetTime << endl;
  _nNoProcFrame = GetPrivateProfileInt(sModuleName.c_str(), "NoProcFrame", 50, sIniFileName.c_str());
  _nScreenWidth = GetPrivateProfileInt(sModuleName.c_str(), "ScreenWidth", 1024, sIniFileName.c_str());

  _bDrawGrid = (GetPrivateProfileInt(sModuleName.c_str(), "DrawGrid", 0, sIniFileName.c_str()) != 0);
  _bDrawViewer = (GetPrivateProfileInt(sModuleName.c_str(), "DrawViewer", 1, sIniFileName.c_str()) != 0);

  _dGridSize = GetPrivateProfileDouble(sModuleName.c_str(), "GridSize", 100.0, sIniFileName.c_str());
  _dPolarGridMaxLen = GetPrivateProfileDouble(sModuleName.c_str(), "PolarGridMaxLen", 20*1000, sIniFileName.c_str());

//  _nTrackerType = GetPrivateProfileInt(sModuleName.c_str(), "TrackerType", 0, sIniFileName.c_str());
  _bChaseLRF = (GetPrivateProfileInt(sModuleName.c_str(), "ChaseLRF", 0, sIniFileName.c_str()) != 0);

  GetPrivateProfileString(sModuleName.c_str(), "FeatureDir", _sFeatureDir.c_str(), sBuf, nBufSize, sIniFileName.c_str());
  _sFeatureDir = sBuf;

  _nIniterType = GetPrivateProfileInt(sModuleName.c_str(), "IniterType", -1, sIniFileName.c_str());
  if (_nIniterType < 0) {
    ccout->SetColor(ColorCout::eGreen);
    ccout << "initer type not set" << endl;
    bool bUsePolarInitializer = (GetPrivateProfileInt(sModuleName.c_str(), "UsePolarInitializer", 1, sIniFileName.c_str()) != 0);
    if (bUsePolarInitializer) {
      ccout << "using polar" << endl;
      _nIniterType = 1;
    }
    else {
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
    else {
      ccout << " warning: unknown grid!" << endl;
    }
  }

  _nHistorySize = GetPrivateProfileInt(sModuleName.c_str(), "HistorySize", 1000, sIniFileName.c_str());

  _bWriteLRFLog = (GetPrivateProfileInt(sModuleName.c_str(), "WriteLRFLog", 0, sIniFileName.c_str()) != 0);
  _bWriteTrackerLog = (GetPrivateProfileInt(sModuleName.c_str(), "WriteTrackerLog", 0, sIniFileName.c_str()) != 0);

  _vGridRange.clear();
  for (int i=0; i<nLRFNum; ++i) {
    double dXMax = GetPrivateProfileDouble(sModuleName.c_str(), (string("XMax")+boost::lexical_cast<string>(i)).c_str(), 10000, sIniFileName.c_str());
    double dXMin = GetPrivateProfileDouble(sModuleName.c_str(), (string("XMin")+boost::lexical_cast<string>(i)).c_str(), -10000, sIniFileName.c_str());
    double dYMax = GetPrivateProfileDouble(sModuleName.c_str(), (string("YMax")+boost::lexical_cast<string>(i)).c_str(), 10000, sIniFileName.c_str());
    double dYMin = GetPrivateProfileDouble(sModuleName.c_str(), (string("YMin")+boost::lexical_cast<string>(i)).c_str(), -10000, sIniFileName.c_str());
    BoostVec v1(2); v1(0) = dXMin; v1(1) = dYMin;
    BoostVec v2(2); v2(0) = dXMax; v2(1) = dYMax;
    _vGridRange.push_back(make_pair(v1, v2));
  }

  //NewTracker用
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
//    sModuleName.c_str(), "InitMinDistFromOtherThrPerson", 500, sIniFileName.c_str());
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
