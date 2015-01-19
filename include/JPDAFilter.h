#pragma once

#include "ParticleFilter.h"
#include "MOTracker.h"
#include <map>
#include <boost/numeric/interval.hpp>
#include <boost/unordered_set.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/noncopyable.hpp>

struct SJPDAFilterData {
  boost::shared_ptr<const CLaserData> _pLaserData;
  std::vector<int> _vExtractedIDs;
  std::vector<int> _vPointStatus; //0: 背景 1: 前景 2: 中間点
};


class CJPDAFilter : public CParticleFilterImpl<SJPDAFilterData>, boost::noncopyable
{
  typedef CParticleFilterImpl<SJPDAFilterData> P;
public:

  CJPDAFilter(size_t nParticleNum, size_t nStateDim=5);
  virtual ~CJPDAFilter();
  virtual void SetTimeStep(double dTimeStep) = 0;

  virtual void Predict(const SJPDAFilterData &rData);
  virtual const BoostVec& Update(const SJPDAFilterData &rData, const std::vector<std::pair<double, int>> &rvBeta);

  virtual const BoostVec& GetResult() const {return _vResult;}
  const BoostVec& GetPredictResult() const {return _vPredictResult;}
  typedef boost::numeric::interval_lib::checking_base<int> P2;
  typedef boost::numeric::interval<int, 
                                   boost::numeric::interval_lib::policies
                                     <boost::numeric::interval_lib::rounded_math<int>,
                                      P2> > RangeInterval;

  const RangeInterval &GetMatchedRangeID() const {return _SeveralMatchedRangeID;}
  const boost::unordered_set<int> &GetMatchedRangeSet() const {return _vMatchedRangeSet;}

  int GetPFID() const {return _nPFID;}

  const std::vector<double> &GetWeights() const {return _vWeight;}
  const std::vector<CPolarVec> &GetParticleCenters() const {return _vParticleCenters;}

  const std::vector<double> &GetRangeSetWeightTotals() {return _vdRangeSetWeightTotals;}

  void ProcessHypo(const SJPDAFilterData &rData, const std::vector<std::vector<RangeInterval>> &rvHypos,
    const boost::dynamic_bitset<> &rPointCorreBitsGroup);

  //戻り値：Pの値とHypoの番号(Updateフェイズで再利用)
  std::pair<double,int> CalcP(const std::vector<RangeInterval> &rHypo);
  void PrepareMemories(size_t nHypoNum);

  const std::vector<double> &GetPTotal() const {return _vdPTotal;}

  struct SLaserStatus {
    int nAngleRangeID;           //LRFのレーザー番号
    double dLaserRange;               //スキャン点計測値
    int nPointStatus;            //0:静止 1:移動 2:誤検出

    bool bInParticleRange;       //パーティクルの範囲内かどうか
    double dParticlePointRange;  //パーティクル点のLRFからの距離

    double dDistFromParticle;   //パーティクルとスキャン点の距離の2乗
    double dLikelihoodInHypo;    //仮説範囲内だった時の尤度
    double dLikelihoodOutofHypo; //仮説範囲外だった時の尤度
    double dLikelihoodNotExist;  //パーティクルが存在しなかった時の尤度

    enum EStatusInRange {
      eMatch, eFP, eStaticOcc, eStaticPT, eMatchFar, eNOP, eInRangeTotal
    };
    enum EStatusOutofRange {
      eDynamicOcc, eDynamicPT, eStaticOcc2, eStaticPT2, eNOP2, eOutofRangeTotal
    };

    EStatusInRange nInRangeStatus;
    EStatusOutofRange nOutofRangeStatus;

    SLaserStatus () {
      nAngleRangeID = -1;
      dLaserRange = 0;
    }
    //debug用
    static const std::string &GetLaserStatusStr(size_t n);
    static std::vector<std::string> s_vLaserString;
  };


  struct SParticleDebugInfo{
    SParticleDebugInfo() {
      nRangeMax = -1;
      nRangeMin = -1;
      nMatchRangeMin = INT_MAX;
      nMatchRangeMax = -INT_MAX;
      vnPointStatus = std::vector<int>( (size_t)SLaserStatus::eInRangeTotal+(size_t)SLaserStatus::eOutofRangeTotal, 0);
    }
    std::vector<int> vnPointStatus;

    int nRangeMin; //パーティクルの範囲の最大値
    int nRangeMax;
    int nMatchRangeMin; //Matchedになった範囲の最大値
    int nMatchRangeMax;
  };

  //_vParticleDebugInfo[nParticleNum][nHypoNum] でデータ取り出し
  const std::vector<std::vector<SParticleDebugInfo>> &GetParticleDebugInfo() const {return _vParticleDebugInfo;}


  void CalcLikelihoodOnRange(const SJPDAFilterData &rData, const std::vector<std::pair<int, int> > &rvRanges);
  const std::vector<std::vector<double> > &GetParticleWeights() const {return _vParticleWeights;}

  void SetSeveralMatchedRangeThr(int n) {_nSeveralMatchedRangeThr = n;}
  int GetSeveralMatchedRangeThr() const {return _nSeveralMatchedRangeThr;}

  /*
  //サイズは3, 0:Match 1:Occ 2:Pass
  const std::vector<double>& GetStatusVector() const {
    return _vStatusVector;
  }
  */

  //位置だけ計算
  boost::tuple<CAngle, double, double> CalcErrorEllipse() const;
  void MakeCovarianceMatrix(BoostMat &rResult) const;

  double CalcDebugMemoryTotal();

  const boost::dynamic_bitset<>& GetCorreBits() const {
    return _PointCorreBits;
    }

  //ついでにここでSLaserStatusを作る
  void SetPointCorreBitsGroup(const boost::dynamic_bitset<> &rPointCorreBitsGroup);

  //Predict particleのLRFまでの平均距離
  double GetAverageDistFromLRF() const {
    return _dAverageDistFromLRF;
  }
  
  bool CrushWithLRF() const {return _bCrushWithLRF;}
  int GetInterval() const {return _nInterval;}
  void SetInterval(int n) {_nInterval = n;}
  const std::vector<RangeInterval> &GetHypoFromID(size_t nHypoID) {
    return _vHypos.at(nHypoID);
  }
  const std::vector<std::pair<double, int>> &GetBeta() const {return _vBeta2;}

  void SetExistanceRate(double d) {_dExistanceRate = d;}
  double GetExistanceRate() const {return _dExistanceRate;}

  bool IsSplittable() const {return _bSplittable;}
  void SetSplittable(bool b) {_bSplittable = b;}

  static bool GetUseRemovedHypoInWeighting() {return s_bUseRemovedHypoInWeighting;}
  static void SetUseRemovedHypoInWeighting(bool b) {s_bUseRemovedHypoInWeighting = b;}

  static void SetFalsePositiveParams(double dFPFarProb, double dDetectLenMax) {
    _dFPFarProb = dFPFarProb;
    _dDetectLenMax = dDetectLenMax;
  }
  static double GetFalsePositiveProbDensity() {
    return _dFPFarProb/_dDetectLenMax;
  }
  //0以下の値を入れると現時点でのパラメータを維持
  void SetLikelihoodFunctionParams(double dSigma, double dOtherMOLenAve, double dOtherMOLenStdDev,
    double dFPExistLen, double dFPNearProb, double dPassProbTotal);


protected:

//  virtual void CalcRange(const SJPDAFilterData &rData, int nPFNum, int &rnMin, int &rnMax, std::vector<double> &rvnDistance) = 0;
  //rvCorrelations: LRFのIDと距離
  virtual bool CalcParticleCorrelation(const SJPDAFilterData &rData, int nPFNum, std::vector<std::pair<int, double>> &rvCorrelations) = 0;

  virtual void Process(BoostVec& rvX, const BoostVec& rvXBefore) = 0;

//  virtual void Weighting(const SJPDAFilterData &rData, const std::vector<double> &rvBeta);
  virtual void Resampling(const SJPDAFilterData &rData);
  virtual void CalcResult();
  virtual double Likelihood(const BoostVec &rX, const SJPDAFilterData &rData);

  void UpdateRangeGroup(const SJPDAFilterData &rData);

  virtual double GetParticleLikelihood(size_t nPFNum, const std::vector<RangeInterval> &rHypo, SParticleDebugInfo &rDebugInfo);
  virtual double GetParticleLikelihood(size_t nPFNum, const std::vector<RangeInterval> &rHypo);

  double _dTimeStep;
  int _nPFID;

  RangeInterval _MatchedRangeID;                //MatchedになったLRF点の範囲 Occ/PTで分裂している場合でもすべてを包含する範囲にする 
  boost::unordered_set<int> _vMatchedRangeSet;  //MatchedになったLRF点のリスト
  RangeInterval _SeveralMatchedRangeID;         //閾値以上の個数の点がMatchedになったLRF点の範囲 分断点の探索はこちらを使う
  RangeInterval _ParticlePointRange;            //全Particle点の範囲


  int _nSeveralMatchedRangeThr; //上の閾値 これ以上の個数のPFからMatched判定を受けた点を_SeveralMatchedRangeIDの対象とする

  std::vector<std::vector<SLaserStatus> > _vParticleLaserStatus; //各PFとレーザーとの対応
  std::vector<std::vector<std::pair<int, double>> > _vParticleCorrelations; //first: LRF点の番号，second: 仮想点のLRFからの距離
  std::vector<CPolarVec> _vParticleCenters; //中央の角度を保存 ＊ローカル座標
  std::vector<std::vector<double> > _vParticleWeights;
  std::vector<std::vector<SParticleDebugInfo>> _vParticleDebugInfo;
  std::vector<double> _vdPTotal;

  std::vector<double> _vdIndependentWeights; //各パーティクルのP(z|x)の値
  std::vector<double> _vdRangeSetWeightTotals; //各サブセットごとのP(z|x)の値の和 sizeは_mAngleRangeSetと同じ

  /* 親クラス
  size_t _nParticleNum;
  size_t _nStateDim;
  std::vector<BoostVec> _avPredict;
  std::vector<BoostVec> _avFilter;
  std::vector<double> _vWeight;

  BoostVec _vProcessVector;
  BoostVec _vSystemNoise;
  BoostVec _vMeasurementNoise;

  BoostVec _vResult;
  */

  BoostVec _vPredictResult;

  static int _nTotalPFNum;

  std::vector<double> _vStatusVector; //サイズは3, 0:Match 1:Occ 2:Pass

  //対象の番号のLRFスキャンとこのPFの間に相関がある
  boost::dynamic_bitset<> _PointCorreBits;
  //対象の番号のLRFスキャンとこのPFの属するグループと相関がある
  boost::dynamic_bitset<> _PointCorreBitsGroup;
  SLRFProperty _LRFProperty;

  bool IsPointInHypo(int nID, const RangeInterval &rRange);

  bool _bCrushWithLRF;
  double _dAverageDistFromLRF;

  //仮説番号をインデックスとして全パーティクルの尤度を格納
  std::vector<boost::shared_ptr<std::vector<double> >> _vpPBuffer;
  //仮説番号->仮説への変換を格納
  std::vector<std::vector<RangeInterval> > _vHypos;
  std::vector<std::pair<double, int>> _vBeta2;
  std::vector<double> _vPValues;

  int _nInterval; //計算量低減のための間引き量

  SJPDAFilterData _CurrentData;

  //パーティクルまでの最短距離を返す
  virtual double GetNearestDistFromParticle(int nPFNo, double x, double y, const boost::shared_ptr<const CCascadedCoords> &pLRFCo) = 0;

  double _dExistanceRate;
  bool _bSplittable; //分割した仮説を作るかどうか

  static bool s_bUseRemovedHypoInWeighting;

  double _dMatchDistThr;    //この距離以内ならParticleとスキャン点に相関があるとみなす
  double _dSigma;           //Matchedの計算に用いる
  double _dOtherMOLenAve;   //DynamicOccのペナルティに用いる他の移動体の厚みの平均
  double _dOtherMOLenStdDev;//DynamicOccのペナルティに用いる他の移動体の厚みの標準偏差
  double _dFPExistLen;      //移動体近くのFPが存在する領域の距離
  double _dFPNearProb;      //移動体近くのFPの発生率
  double _dPassProbTotal;   //全部の点が見えなくなる確率
  static double _dFPFarProb;//移動体と関係ないFPの発生率．全移動体で共通なのでstatic. 
  static double _dDetectLenMax; //この範囲内でのみ検出を行う
};
