#pragma once

#include <vector>
#include <string>
#include "Coordinates.h"
#include "LRFSource.h"
#include "PolygonalRegion.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "AppProperties.h"

struct STrackerConfig {

  STrackerConfig(){}
  virtual ~STrackerConfig(){}
  void LoadFromIni(const std::string &sFileName);

  //LRF
  int _nMode; //0:File 1:URG 2:SICK 3:ROS
  std::vector<CCoordinates2D> _vLRFCos; //LRF座標

  std::vector<SLRFProperty> _vLogLRFProperty; //mode0 LRFのプロパティを格納
  std::string _sOdmFileName; //mode0 オドメトリのファイル名 
  std::vector<std::string> _vsFileName; //mode0 ファイル名
  double _dStartTime; //mode0 開始時刻
  double _dEndTime; //mode0 終了時刻

  std::vector<int> _vnPorts; //mode1,2 ポート番号
  std::vector<std::string> _vsAddrName; // mode 2 IPアドレス
  std::vector<int> _vnFogMode; //mode2 霧モード
  std::vector<int> _vnNPalse;  //mode2 Nパルスモード

  bool _bWaitForTransform; //mode3(ROS) LRF取得時にWaiftforTransformを行う

  //Tracker Property
  //int _nTrackerType;
  bool _bAuto; //追跡処理自動スタート
  int _nNoProcFrame; //始まってからこのフレーム数は追跡処理しない
  int _nHistorySize; //巻き戻しに使う履歴のサイズ
  double _dFrameUpdateMinTime; //実機使用の場合のみ,1フレームの最低秒数．デフォルト0.095(安定して10FPSにするため)
  int _nMaxTargetNum; //最大追跡対象数

  double _dProcResetTime; //これ以上の時間がかかったらリセットする
  std::vector<CPolygonalRegion> _vDetectRegion; //この範囲内にある点のみを対象とする．特に指定しなければ_vGridRangeと同じ
  std::vector< std::pair<BoostVec, BoostVec > > _vGridRange; //初期描画範囲＋グリッドの領域．Polarモードでは初期描画領域のみ

  //log
  bool _bWriteLRFLog;
  bool _bWriteTrackerLog;


};

