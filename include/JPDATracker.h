#pragma once
#include "SJPDATrackerConfig.h"
#include "JPDAFilter.h"
#include <boost/unordered_map.hpp>
#include <boost/noncopyable.hpp>

class CTrackInitializationWithProperty;
class CMOClassification;
//class CSVMDataProcessing;

class CJPDATracker :
  public CMOTracker
{
public:
  CJPDATracker(const SJPDATrackerConfig &rConfig);
  virtual ~CJPDATracker(void);

  virtual boost::shared_ptr<const STrackerResult> Proc(const LaserDataBuffer &rvLasers);

  //移動物体初期化候補となるクラスタ 追跡だけならクラスタリングの必要なし
  typedef enum {
    Bulge,     //両端が奥
    Wall,      //両端が手前
    Occluding1,//手前->奥
    Occluding2,//奥->手前
  } SegType;

  struct SPointCluster {
    std::vector<boost::shared_ptr<CCascadedVec> > _vPoints;
    std::vector<double> _vDistances;
    boost::numeric::interval<CAngle> _AngleRange;
    CJPDAFilter::RangeInterval _PointIDRange;
    double _dTotalLen;
    double _dAveDist;
    double _dDistBetweenEdge; //両端の距離
    double _dWidthFromLRF; //_dAveDist*(sin(両端の角度差/2)*2
  };

  struct SPFStatus {
    BoostVec _vAverage;
    std::vector<BoostVec> _vParticles;
    std::vector<BoostVec> _vParticlesPredict;
    std::vector<double> _vdLikelihood;
    size_t _nClusterID;
    std::string _sType;

    boost::numeric::interval<CAngle> _AngleRange; //PFの拡散
    std::vector<size_t> _vnGoodPoints;    //尤度が良い順に数個
  };

  boost::shared_ptr<const CTrackInitializationWithProperty> GetInitializer() const {
    return _pInitializer;
  }

  virtual void ResetAll();
  virtual void SetPerformTracking(bool bPerformTrack);

  void SetIntervalStandardLen(double d) {
    _dIntervalStandardLen = d;
  }

  //ConstructHypo関数で使うための構造体
  struct SHypothesis2 {
    std::vector<int> _vnPos; //CBetaCalculateHelperの位置
    double _dLikelihood;
  };
  //汎用
  struct SHypothesis {
    struct SCorrespondance {
      boost::shared_ptr<const CJPDAFilter> _pFilter;
      std::vector<CJPDAFilter::RangeInterval> _vRanges;
    };
    std::vector<SCorrespondance> _vCorrespondance;
    double _dLikelihood;
  };

  void SetUseJIPDA(bool b);
  bool GetUseJIPDA() const {return _bUseJIPDA;}
  bool GetUseDynamicProgramming() const {return _bUseDynamicProgramming;}
  void SetUseDynamicProgramming(bool b) {_bUseDynamicProgramming = b;}

protected:

  struct SPFCluster : boost::noncopyable {

    SPFCluster(const boost::dynamic_bitset<> &rvCorr, std::vector<boost::shared_ptr<CJPDAFilter> > &rvpFilters, int nLRFPointNum) {
      _nInterval = 1;
      ConstructGroup(rvCorr, rvpFilters, nLRFPointNum);
    }
    SPFCluster(const boost::dynamic_bitset<> &rvCorr, const SPFCluster& rOriginal) {
      _nInterval = rOriginal._nInterval;
      _nLRFPointNum = rOriginal._nLRFPointNum;
      ConstructGroup(rvCorr, rOriginal._vFilterSorted, _nLRFPointNum);
    }
    ~SPFCluster() {}

    //_v2nOrdersに順序を生成
    void BuildOrders();

    int GetInterval() const {return _nInterval;}
    void SetInterval(int n);

    const CJPDAFilter::RangeInterval & GetGroupRange() const {return _GroupRange;}
    const CJPDAFilter::RangeInterval & GetGroupRangeOrg() const {return _GroupRangeOrg;}
//    const std::vector<CJPDAFilter::RangeInterval> &GetPFCoverRange() const {return _vPFCoverRange;}
    const std::vector<boost::shared_ptr<CJPDAFilter> > &GetFilters() const{return _vFilterSorted;}
    std::vector<boost::shared_ptr<CJPDAFilter> > &GetFilters() {return _vFilterSorted;}
    const std::vector<CJPDAFilter::RangeInterval> &GetCoverRanges() const {return _vCoverRangeSorted;}
    const std::vector<std::vector<size_t> > &GetOrders() const {
      return _v2nOrders;
    }
    const boost::dynamic_bitset<> &GetCorrelation() const {return _vCorrelation;}
    bool IsStrideBorder() const {return _bStrideBorder;}

    static void SetUseJIPDA(bool b) {s_bUseJIPDA = b;}
    static bool GetUseJIPDA() {return s_bUseJIPDA;}
    static void SetShowMsg(bool b) {s_bShowMsg = b;}
    static bool GetShowMsg() {return s_bShowMsg;}

    size_t GetTotalValidLRFPointsNum(const SJPDAFilterData& rCurrentFilterData) const;
  protected:

    bool IsOrderOK(const std::vector<size_t> &rOrder);
    void ConstructGroup(const boost::dynamic_bitset<> &rvCorr, const std::vector<boost::shared_ptr<CJPDAFilter> > &rvpFilters, int nLRFPointNum);
    void ConstructOrders(std::vector<size_t> vnCurrent, std::vector<size_t> vnNotAdded);
    void ConstructOrdersNew(std::vector<size_t> vnCurrent, std::vector<size_t> vnNotAdded);
    int _nInterval;
    int _nLRFPointNum;

    bool _bStrideBorder; //境界線をまたいでいるかどうか
    std::vector<boost::shared_ptr<CJPDAFilter> > _vFilterSorted;
    //これら2つは_nIntervalの境界に合うように加工済み + 一周判定もあり
    CJPDAFilter::RangeInterval _GroupRange; //_nIntervalの境界に合うように加工済み
    CJPDAFilter::RangeInterval _GroupRangeOrg; //もともとの範囲
    std::vector<CJPDAFilter::RangeInterval> _vCoverRangeSorted;  

    boost::dynamic_bitset<> _vCorrelation; //グループ内のPFが相関する点の情報
    std::vector<std::vector<size_t> > _v2nOrders; //グループ内のありうる順序

    static bool s_bUseJIPDA;
    static bool s_bShowMsg;
  };

  class CBetaCalculateHelper : boost::noncopyable{
  public:

    CBetaCalculateHelper (boost::shared_ptr<CJPDAFilter> pFilter, const CJPDAFilter::RangeInterval &rFilterRange, int nInterval, const CJPDAFilter::RangeInterval &rGroupRange) {
      _pPrev = nullptr;
      _pNext = nullptr;
      _FilterRange = rFilterRange;
      _nInterval = nInterval;
      _pFilter = pFilter;
      _GroupRange = rGroupRange;
    }
    ~CBetaCalculateHelper() {}
    void SetTree(CBetaCalculateHelper* pPrev, CBetaCalculateHelper* pNext);
    void CalcBeta();

    void InsertPrev(int nKey, size_t nPos) {
      vPrevs[nKey].push_back(nPos);
    }
    void InsertNext(int nKey, size_t nPos) {
      vNexts[nKey].push_back(nPos);
    }
    double GetLeft(size_t n);
    double GetRight(size_t n);
    int CountHypoNum(int n=-1); //debug用

    const std::vector<double> &GetBeta() const {return _vBeta;}
    void SetBeta(const std::vector<double> &rBeta) {
      _vBeta=rBeta;
    }
    const std::vector<int> &GetBackUpID() const {return _vPBackUpID;}

    const std::vector<double> GetP() const {return _vP;}
    void GetNextPos(size_t n, std::vector<size_t> &rvPos);
    const std::vector<CJPDAFilter::RangeInterval> &GetRanges() const
    {return _vRanges;}

    boost::shared_ptr<const CJPDAFilter> GetFilter() const {return _pFilter;}

    static void SetUseJIPDA(bool b) {s_bUseJIPDA = b;}
    static bool GetUseJIPDA() {return s_bUseJIPDA;}
    static void SetShowMsg(bool b) {s_bShowMsg = b;}
    static bool GetShowMsg() {return s_bShowMsg;}

  protected:

    static bool s_bShowMsg;

    std::vector<CJPDAFilter::RangeInterval> _vRanges; //個別の仮説範囲
    std::vector<double> _vBeta;
    std::vector<double> _vLeft;
    std::vector<double> _vRight;
    std::vector<double> _vP;
    std::vector<int> _vPBackUpID; //Filterのlikelihood使い回しに使うID

    boost::unordered_map<int, std::vector<size_t>> vPrevs; //second: pPrev->vRangesの番号
    boost::unordered_map<int, std::vector<size_t>> vNexts; //second: pNext->vRangesの番号
    boost::shared_ptr<CJPDAFilter> _pFilter;
    CJPDAFilter::RangeInterval _FilterRange; //全パーティクルの範囲
    int _nInterval;
    CBetaCalculateHelper* _pPrev;
    CBetaCalculateHelper* _pNext;

    CJPDAFilter::RangeInterval _GroupRange; //このPFを含むグループ全体の範囲
    static bool s_bUseJIPDA;
  };

  int _nCurrentFrame;
  double _dCurrentTime;
  double _dStepTime;

  boost::shared_ptr<CTrackInitializationWithProperty> _pInitializer;
  SJPDATrackerConfig _CurrentConfig;
  int _nLRFPointNum;

  LaserDataBuffer _CurrentBuffer;

  //step0 最初に必要な処理 _pCurrentLaserData:最新のLRFデータ，_pBeforeLaserData:前フレームのLRFデータ
  void Initialize(); //Frame0のみ
  bool PrepareTracking(const LaserDataBuffer &rvLasers);
  boost::shared_ptr<const CLaserData> _pCurrentLaserData;
  boost::shared_ptr<const CLaserData> _pBeforeLaserData;
  SJPDAFilterData _CurrentFilterData;

  //step1 候補領域の抽出 _vExtractedIDs, _vExtractedPointsに結果格納 PFで使いやすいように_vPointStatusも作る
  void ExtractMovingPoints();
  std::vector<int> _vExtractedIDs;
  std::vector<boost::shared_ptr<CCascadedVec> > _vExtractedPoints;
  std::vector<int> _vPointStatus; //0:背景 1:前景 2:中間点

  //step2 PF predict phase
  void PredictPF();

  //step3 仮説生成 + PF更新
  void SolveJPDA();
  void MakeGroup(); //相関のあるPFをまとめる
  std::vector<boost::shared_ptr<SPFCluster> > _vGroupFirst; //first:カバー範囲 second:_vpFiltersの番号
  std::vector<boost::shared_ptr<SPFCluster> > _vGroup; //first:カバー範囲 second:_vpFiltersの番号
  //仮説列挙でBetaを計算する．戻り値：最尤仮説
  boost::shared_ptr<SHypothesis> CalcBetaUsingEnumeration(std::vector<boost::shared_ptr<CBetaCalculateHelper> > &rvpBeta);
  //CalcBetaUsingEnumerationで使う仮説列挙用関数
  void ConstructHypo(std::vector<size_t> vHypo, const std::vector<boost::shared_ptr<CBetaCalculateHelper> > &rvpBeta,   std::vector<boost::shared_ptr<SHypothesis2> > &rvResults);
  //動的計画法を使った場合に最尤仮説を求める関数
  boost::shared_ptr<SHypothesis> CalcBestHypo(std::vector<boost::shared_ptr<CBetaCalculateHelper> > &rvpBeta);  

  //step4 追跡終了したPFを消去
  void RemoveTerminatedPF();

  //step5 追跡に使用されなかった候補領域をまとめる _vExtractedClustersに結果格納
  void MakeCluster();
  std::vector<boost::shared_ptr<SPointCluster> > _vExtractedClusters;
  std::vector<std::vector<int> > _vClusterGroup; //_vExtractedClustersについて，距離の近いものをグループ化

  //step6 移動体の候補初期化
  void InitializeNewObjects();
  std::vector<boost::shared_ptr<CJPDAFilter> > _vpFilters;

  //衝突がある移動体へのポインタを返す
  boost::shared_ptr<CJPDAFilter> CheckCollision(boost::shared_ptr<CJPDAFilter> pFilter, std::string &rsErrorMsg);
  boost::shared_ptr<const STrackerResult> _pCurrentResult;

  std::vector<double> _vBestHypoDivLines;

  bool _bUseEllipse;
  int _nParticleNum;

  struct SEventLogger :boost::noncopyable {
    SEventLogger() {
      _pos = &std::cout;
    }
    void SetFile(const std::string &rsFileName) {
      _pFile.reset(new std::ofstream(rsFileName.c_str()));
      _pos = _pFile.get();
    }
    void Log(const std::string &rsWhat) {
      (*_pos) << rsWhat;
    }
    std::ostream *_pos;
    boost::shared_ptr<std::ofstream> _pFile;
  };

  SEventLogger _Logger;
  void WriteEventLog(const std::string &rsEvent, const std::string &rsDetail);

  boost::recursive_mutex _ProcMutex;
  bool _bPerformTracking;
  
  void DumpCSV();
  double _dIntervalStandardLen; //この距離のときを基準に間引く 0以下:間引きなし
  bool _bUseDynamicProgramming;
  bool _bUseJIPDA;
  bool _bCalcBestHypoInDPMode;
  std::vector<double> _vExtractedPointsScore;

  boost::shared_ptr<CMOClassification> _pClassifier;

  //Remover
  double _dRemoveExistanceRateThr;
  double _dRemoveDisparsionThr;

  bool _bPredictMT;

//  boost::shared_ptr<CSVMDataProcessing> _pSVMDataProcessing;

  double _dMinClusterDivideThr;

};


struct SJPDATrackerResult : public STrackerResult {

  virtual ~SJPDATrackerResult() {}

  std::vector<boost::shared_ptr<CCascadedVec> > _vExtractedPoints;
  std::vector<boost::shared_ptr<CJPDATracker::SPointCluster> > _vExtractedClusters;

  std::vector<double> _vBestHypoDivLines;
  std::vector<boost::shared_ptr<CJPDATracker::SPFStatus> > _vTempPFResults;

  std::vector<int> _vPointStatus; //0: 背景 1: 前景 2: 中間点

  SJPDATrackerConfig _Config;
};

