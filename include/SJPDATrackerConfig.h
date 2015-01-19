#pragma once

#include "TrackerConfig.h"

class SJPDATrackerConfig : public STrackerConfig
{
public:
  SJPDATrackerConfig(void) {}
  virtual ~SJPDATrackerConfig(void) {}

  virtual void LoadFromIni(const std::string &sFileName);

  //viewer
  bool _bDrawViewer;
  bool _bChaseLRF; //trueにすると，処理が行われるたびにLRFを中央に表示する
  double _dViewerSizeY;
  double _dViewerRange;
  size_t _nScreenWidth;

  //initializer
  bool _bDrawGrid; //Initializer用グリッドの描画
  int _nIniterType; //0:グリッド 1:極座標グリッド 2:軌跡グリッド
  bool _bUpdateGrid;
  int _nMaxUpdateGridFrame;
  double _dObsThr;
  double _dFreeThr;
  double _dLRFObsOdds;
  double _dLRFFreeOdds;
  double _dLRFOddsMax;
  double _dLRFOddsMin;
  double _dGridSize;
  std::vector<std::string> _vsSaveGridFileNames;
  std::vector<std::string> _vsLoadGridFileNames;
  double _dPolarGridMaxLen;

  //tracker
  bool _bPerformCarTrack;
  std::string _sFeatureDir; //SVM用のファイル
  double _dRemoveIntervalDistThrMax;
  double _dRemoveIntervalDistThrMin;
  double _dRemoveIntervalDistMulti;
  double _dClusterLenThr;
  double _dClusterAngleThr; //degreeで指定
  double _dNewObjPointExtractThr;
  double _dInitTotalLenThr;
  double _dInitWidthTotalThr;
  double _dInitDistBeteenEdgeThr;
  double _dInitMinDistFromOtherThrPerson;
  bool _bDumpDebugInfo;
  bool _bShowDebugString;
  double _dInitDistanceThr;
  bool _bShowHypoLines;
  bool _bUseDynamicProgramming;
  bool _bUseJIPDA;
  bool _bSetPersonSplittable;
  bool _bUseRemovedHypoInWeighting;
  double _dRemoveExistanceRateThr;
  double _dRemoveDisparsionThr;
  double _dIntervalStandardLen;
  bool _bWriteEventLog;
  bool _bDumpSVMData;

  double _dPersonR1Ave;
  double _dPersonR1StdDev;
  double _dPersonR1Max;
  double _dPersonR1Min;
  double _dPersonR2Ave;
  double _dPersonR2StdDev;
  double _dPersonR2Max;
  double _dPersonR2Min;
  double _dPersonXN;
  double _dPersonRadN;
  double _dPersonVN1;
  double _dPersonRVN1;
  double _dPersonVN2;
  double _dPersonRVN2;
  double _dPersonLowVelThr;
  double _dPersonRVNLowVel;

  double _dPersonSigma;
  double _dPersonOtherMOLenAve;
  double _dPersonOtherMOLenStdDev;
  double _dPersonFPExistLen;
  double _dPersonFPNearProb;
  double _dPersonPassProbTotal;

};
