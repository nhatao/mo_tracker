#include "StdAfx_MOTracking.h"
#include "TrackerConfig.h"
#include "ColorCout.h"
#include <boost/filesystem.hpp>
using namespace std;

void STrackerConfig::LoadFromIni(const std::string &sIniFileName2) {

  if (!boost::filesystem::exists(sIniFileName2)) {
    ESStreamException ess; ess << sIniFileName2 << " not exist";
    throw ess;
  }
  boost::filesystem::path logfile(sIniFileName2);
  string sIniFileName = boost::filesystem::absolute(logfile).string();
  const int nBufSize = 2048;
  char sBuf[nBufSize];
  string sModuleName = "PT";
  _nMode = GetPrivateProfileInt(sModuleName.c_str(), "Mode", -1, sIniFileName.c_str());
  int nLRFNum = GetPrivateProfileInt(sModuleName.c_str(), "LRFNum", 1, sIniFileName.c_str());
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

  _dFrameUpdateMinTime = GetPrivateProfileDouble(
    sModuleName.c_str(), "FrameUpdateMinTime", 0.095, sIniFileName.c_str());
 
  if (_nMode == 0) {
    GetPrivateProfileString(sModuleName.c_str(), "OdmFileName", "", sBuf, nBufSize, sIniFileName.c_str());
    _sOdmFileName = sBuf;
  }

  _nMaxTargetNum = GetPrivateProfileInt(sModuleName.c_str(), "MaxTargetNum", 30, sIniFileName.c_str()); 
  _dStartTime = GetPrivateProfileDouble(sModuleName.c_str(), "Log_FirstTime", 0.0, sIniFileName.c_str());
  _dEndTime = GetPrivateProfileDouble(sModuleName.c_str(), "Log_EndTime", DBL_MAX, sIniFileName.c_str());
  _dProcResetTime = GetPrivateProfileDouble(sModuleName.c_str(), "ProcResetTime", -1, sIniFileName.c_str());
  cout << "ResetTime: " << _dProcResetTime << endl;
  _nNoProcFrame = GetPrivateProfileInt(sModuleName.c_str(), "NoProcFrame", 50, sIniFileName.c_str());
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

}

