#pragma once

#include "SerialControl.h"
#include <string>
#include <fstream>
#include <boost/utility.hpp>
#include <iostream>
#include "Coordinates.h"
#include "mmtimer.h"
#include "Angle.h"

struct SLRFProperty {

  SLRFProperty() {
    _nElemNum = 1081;
    _dReso.set_deg(0.25);
    _dFirstAngle.set_deg(-135);
    _bInverse = false;
    _bModeMM = true;
    _dMinRange = 1;
    _dMaxRange = 30*1000;
    _dScanTime = 0.025;
    _dTimeIncrement = 0.025/(360/0.25);
  }

  unsigned int _nElemNum; //LRF点数
  bool _bModeMM;    //単位がmmならtrue, cmならfalse ややこしいのでobsolete
  CAngle _dReso;    //解像度
  CAngle _dFirstAngle;//0番目のLaserの角度
  bool _bInverse;   //上下逆に設置もしくはレーザー回転方向が逆

  double _dMinRange; //単位はmm
  double _dMaxRange; //単位はmm
  double _dScanTime; //1フレームあたりの時間(s)
  double _dTimeIncrement; //1点あたりの時間(s) skew補正などに使う

  CAngle IndexToAngle(unsigned int n) const {
    if (n>=_nElemNum) {
      std::ostringstream oss;
      oss << __FUNCTION__ << " n=" << n;
      throw std::out_of_range(oss.str().c_str());
    }
    return CAngle(_dFirstAngle.get()+_dReso.get()*n);
  }
  CAngle IndexToAngleWithoutException(unsigned int n) const {
    return CAngle(_dFirstAngle.get()+_dReso.get()*n);
  }

  unsigned int AngleToIndex(const CAngle& rAngle) const {
    int n = (int)round(((rAngle-_dFirstAngle).get_positive())/(_dReso.get()));
    if ((n < 0) || (n >= (int)_nElemNum)) {
      std::ostringstream oss;
      oss << __FUNCTION__ << " Angle=" << rAngle.get_deg() << " n=" << n;
      throw std::out_of_range(oss.str().c_str());
    }
    return (unsigned int)n;
  }
  unsigned int AngleToIndexWithoutException(const CAngle& rAngle) const {
    return (unsigned int)round(((rAngle-_dFirstAngle).get_positive())/(_dReso.get()));
  }


  double AngleToContinuousIndex(const CAngle& rAngle) const { //あえて連続値でindexを返す
    double n = ((rAngle-_dFirstAngle).get_positive())/(_dReso.get());
    return n;
  }
  CAngle GetLastAngle() const {
    return CAngle(_dFirstAngle.get()+_dReso.get()*(_nElemNum-1));
  }

};

class CLRFSource
{
public:
  virtual ~CLRFSource() {
    delete[] _aData;
    delete[] _aCurrentRawData;
  };

  const unsigned short *ReadData() const;
  virtual void Measure() = 0;
  virtual void RequestData() {}; //呼び出しはRequestData->Measureの順．RequestDataは可能な限り一瞬で終わるコマンド，Measureは時間がかかってもよい
  virtual void Reset() = 0;
  virtual bool CheckInvalidData(unsigned short nData) const = 0; //センサ固有のエラー値の場合falseを返す
  virtual double GetCurrentScanTime () const {return 0;} // 最後のスキャンが入ってきたときのハードウェア時間(sec) サポートしない場合は0のまま
  
//  void SetData(CLRFData& rLRFData) const;

  bool IsModeMM() const {return _LRFProperty._bModeMM;}
  virtual void SetModeMM(bool bMM) {_LRFProperty._bModeMM = bMM;} //ハードウェア的な操作が必要ならオーバーロード

  int GetElemNum() const {return (_nLastPos-_nFirstPos-1)/(_nInterval+1)+1;}
  double GetReso() const {return (_LRFProperty._dReso.get_deg())*(_nInterval+1);}
  double GetFirstDeg() const {return
    (_LRFProperty._dFirstAngle.get_deg() + 
    _nFirstPos*_LRFProperty._dReso.get_deg()); }

  double GetLastDeg() const {return GetFirstDeg()+GetReso()*(GetElemNum()-1);}
  int GetFirstPos() const {return _nFirstPos;}
  int GetLastPos() const {return _nLastPos;}

  void SetInverse(bool bInverse) {_LRFProperty._bInverse = bInverse;}

  void SetPosLimit(int nFirst, int nLast) { //nFirstからnLastまで使う 負の値を入れると最大値にセット
    if (nFirst>=0) _nFirstPos = nFirst;
    else _nFirstPos = 0;
    if ((nLast>_nFirstPos) && (nLast<=(int)_LRFProperty._nElemNum)) _nLastPos = nLast;
    else _nLastPos = (int)_LRFProperty._nElemNum;
  }
  const std::string& GetName() const {return _sLRFName;}

  void SetInterval(size_t n) { //間引き量をセット。0なら間引きなし、1なら一つおき
    _nInterval = (int)n;
  }
  size_t GetInterval() const {return _nInterval;}

  const SLRFProperty &GetProperty() const {return _LRFProperty;}

  void SetParam(int nElemNum, double dReso, double dFirstDeg, bool bInverse) {
    _LRFProperty._nElemNum = nElemNum;
    _LRFProperty._dReso.set_deg(dReso);
    _LRFProperty._dFirstAngle.set_deg(dFirstDeg);
    _LRFProperty._bInverse = bInverse;
    _nFirstPos = 0;
    _nLastPos = _LRFProperty._nElemNum;

    _nInterval = 0;
    InitArray();    
  }

protected:
  CLRFSource(void) {
    _nLastPos = (int)_LRFProperty._nElemNum; 
    _nFirstPos = 0; 
    _nInterval = 0;
    _aData = _aCurrentRawData = NULL;
    InitArray();
  }

  CLRFSource(const SLRFProperty &rProperty) {
    _LRFProperty = rProperty;
    _nLastPos = (int)_LRFProperty._nElemNum; 
    _nFirstPos = 0; 
    _nInterval = 0;
    _aData = _aCurrentRawData = NULL;
    InitArray();
  }


  virtual const unsigned short *ReadDataImpl() const {return _aCurrentRawData;} //間引きなどがされていないデータを返す関数．オーバーライドしない場合は_aCurrentRawDataにLRFのデータをコピー
  void InitArray() { //子クラスはこれを必ず呼び出す！！
    delete[] _aData;
    delete[] _aCurrentRawData;
    _aCurrentRawData = new unsigned short[_LRFProperty._nElemNum];
    _aData = new unsigned short[_LRFProperty._nElemNum];
    memset(_aCurrentRawData, 0, _LRFProperty._nElemNum*sizeof(unsigned short));
  }

  int _nInterval; //間引き量
  int _nFirstPos; //全データ中何番目から使うか
  int _nLastPos;  //全データ中何番目まで使うか

  SLRFProperty _LRFProperty;
  
  mutable unsigned short* _aData; //間引き済みデータ
  unsigned short* _aCurrentRawData;
  CCoordinates2D _vCurCoord;
  std::string _sLRFName; // SICK or URG

};

class CLRFFromFile : public CLRFSource, public boost::noncopyable
{
public:
  virtual ~CLRFFromFile(void);
  CLRFFromFile(void);
  
  bool Init(const std::string& sFileName, int nElemNum, double dReso, double dFirstDeg, const std::string &rsLRFName, bool bHasTime, int nFirstLine = 0);
  bool Init(const std::string& sFileName, const SLRFProperty &rProperty, const std::string &rsLRFName, bool bHasTime, int nFirstLine = 0);



  bool InitAsTopURG(const std::string& sFileName, bool bHasTime);
  bool InitAsBlackURG(const std::string& sFileName, bool bHasTime);
  virtual const unsigned short *ReadDataImpl() const;
  virtual void Reset();
  virtual void Measure();
  virtual bool CheckInvalidData(unsigned short nData) const {if (nData>0) return true; else return false;}
  virtual double GetCurrentScanTime () const {return _dCurTime;}

protected:
  std::string _sFileName;
  std::ifstream *_pFin;
  std::string _sLRFName; // sick or urg
  bool _bHasTime;
  double _dCurTime;
  int _nFirstLine; //ファイル何行目から始めるか
};


