#pragma once

#include "OutputDebugStream.h"
#include <vector>
#include "MatrixFuncs.h"
#include <boost/function.hpp>
#include <iostream>
#include "mmtimer.h"
#include <boost/serialization/export.hpp>
#define BOOST_LIB_NAME boost_serialization
#include <boost/config/auto_link.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/version.hpp>
#include <boost/bind.hpp>

#include <iostream>
template <class T>
std::istream& isread(std::istream& is, T& data) {
   return is.read(reinterpret_cast<char*>(&data), sizeof(T));
}
template <class T>
std::ostream& oswrite(std::ostream& os, const T& data) {
   return os.write(reinterpret_cast<const char*>(&data), sizeof(T));
}
template <class T>
std::istream& isread(std::istream& is, T& data, size_t n) {
   return is.read(reinterpret_cast<char*>(&data), n);
}
template <class T>
std::ostream& oswrite(std::ostream& os, const T& data, size_t n) {
   return os.write(reinterpret_cast<const char*>(&data), n);
}

class CCoordinates3D;

//ボクセル・グリッドクラスの要素となる構造体
//CDigitizedSpaceのTには、必ずこの構造体を継承したクラス・構造体を用いる

template <typename VOTE_TYPE> 
struct SSimpleUnitTemplate {
  typedef VOTE_TYPE value_type;
  VOTE_TYPE nVotedNum;
  int nLabel;
  int *aPos;
  std::vector<int> *vnNeighbors;

  SSimpleUnitTemplate() {
    nVotedNum=0;
    nLabel=-1;
    aPos=NULL;
    vnNeighbors = new std::vector<int>;
  }
  virtual ~SSimpleUnitTemplate() {
    delete vnNeighbors;
    delete[] aPos;
  }
  virtual bool IsEnable() const{
    return true;
  }

private:
  SSimpleUnitTemplate(SSimpleUnitTemplate &);
  SSimpleUnitTemplate &operator = (SSimpleUnitTemplate &);
};

typedef SSimpleUnitTemplate<int> SSimpleUnit;

/*
  グリッド・ボクセルクラスの親クラス
  各要素に対しては、ApplyAll(), ApplySpecific()メンバ関数を用いて操作を行う
*/

template <class T, int DIM>
class CDigitizedSpace {

protected:
  CDigitizedSpace() {
    //TをTSSimpleUnitから派生させていないと、ここでコンパイルエラーになる
    _anSpaceSize = NULL;
    _aUnits = NULL;
    _anLabelSize = NULL;
    _nLabelNum=0;
    _nTotalUnitNum=0;
  }
  virtual ~CDigitizedSpace() {
    Clear();
  }
  void Init(int* anSize) {

    Clear();
    _anSpaceSize = new int[DIM];
    _nTotalUnitNum = 1;
    for(int i=0; i<DIM; ++i) {
      _nTotalUnitNum*=anSize[i];
      _anSpaceSize[i]=anSize[i];
    }
    _aUnits = new T[_nTotalUnitNum];
    _anLabelSize = new int[_nTotalUnitNum];
    for(int i=0; i<_nTotalUnitNum; ++i) {
      _aUnits[i].aPos = new int[DIM];
    }
  }
  void Clear() {
    delete [] _anSpaceSize;
    delete [] _aUnits;
    delete [] _anLabelSize;
  }
public:

  void Copy(const CDigitizedSpace<T, DIM> &rhs) {
    delete [] _anSpaceSize;
    delete [] _aUnits;
    delete [] _anLabelSize;
    Init(rhs._anSpaceSize);
    for(int i=0; i<_nTotalUnitNum; ++i) {
      _aUnits[i] = rhs._aUnits[i];
      _aUnits[i].aPos = new int[DIM];
      for (int j=0; j<DIM; ++j) {
        _aUnits[i].aPos[j] = rhs._aUnits[i].aPos[j];
      }
      _aUnits[i].vnNeighbors = new std::vector<int>;
      (*_aUnits[i].vnNeighbors) = (*(rhs._aUnits[i].vnNeighbors));
      _anLabelSize[i] = rhs._anLabelSize[i];
    }
  }

  const T* GetObjects() const {return _aUnits;}
  T* GetObjects() {return _aUnits;}

  typedef boost::function1<void, T&> UnitFunc;
  typedef boost::function1<void, const T&> ConstUnitFunc;
  template <typename F1>
  void ApplyAll(const F1& func) {
    T* pTemp = _aUnits;
    for (int i=0; i<_nTotalUnitNum; ++i) {
      func(*pTemp);
      ++pTemp;
    }
  }
  void ApplySpecific(int n, const UnitFunc& func) { func(_aUnits[n]); }

  //IsSameGroupがtrueを返すときのみ同じグループとみなし、同一のラベルをつける
  typedef boost::function2<bool, T&, T&> IsSameGroupFunc;
  int Labeling(const IsSameGroupFunc &IsSameGroup) {
    int nCurLabel=0;
    T* pCur = _aUnits;
    T* pEnd = _aUnits+_nTotalUnitNum;
    std::vector<std::pair<int, int> > lSameGroup;

    mmtimer mt;

    while (pCur != pEnd) {
      pCur->nLabel = -1;
      ++pCur;
    }
    pCur = _aUnits;
    //初回ラベリング
    while (pCur != pEnd) {
      if (IsSameGroup(*pCur, *pCur)) {
        if (pCur->nLabel == -1) {
          pCur->nLabel=nCurLabel;
          ++nCurLabel;
        }
        typedef std::vector<int>::iterator IT1;
        for (IT1 it=pCur->vnNeighbors->begin(); it!=pCur->vnNeighbors->end(); ++it ) {
//          T* pTarget = pCur + (*it);
          T* pTarget = _aUnits + (*it);
          if (IsSameGroup(*pCur,*pTarget)) {
            if (pTarget->nLabel == -1) {
              pTarget->nLabel=pCur->nLabel;
            }
            else if (pTarget->nLabel!=pCur->nLabel){
              lSameGroup.push_back(std::make_pair( min(pTarget->nLabel, pCur->nLabel), 
                                                   max(pTarget->nLabel, pCur->nLabel) ));
            }
          }
        }
      }
      ++pCur;
    }
    //同じグループをまとめる
    int *pnTable = new int[nCurLabel];
    for (int i=0; i<nCurLabel; ++i) pnTable[i]=-1;
    for (std::vector<std::pair<int, int> >::iterator itl = lSameGroup.begin(); itl != lSameGroup.end(); ++itl) {
      if ((pnTable[itl->first] == -1) && (pnTable[itl->second] == -1)) {
        pnTable[itl->first]=itl->first;
        pnTable[itl->second]=itl->first;
      }
      else if (pnTable[itl->second] == -1) {
        pnTable[itl->second]=itl->first;
      }
      else if (pnTable[itl->first] == -1) {
        pnTable[itl->first]=itl->second;
      }
      else {
        int nSmall, nLarge;
        if (pnTable[itl->first] > pnTable[itl->second]) {
          nLarge = pnTable[itl->first];
          nSmall = pnTable[itl->second];
        }
        else {
          nSmall = pnTable[itl->first];
          nLarge = pnTable[itl->second];
        }
        for (int i=0; i<nCurLabel; ++i) {
          if (pnTable[i] == nLarge) {
            pnTable[i] = nSmall;
          }
        }
      }
    }
    for (int i=0; i<nCurLabel; ++i) {
      if (pnTable[i] == -1) pnTable[i]=i;
    }
    //番号をつめる
    int diff=0;
    for (int i=0; i<nCurLabel; ++i) {
      int cnt=0;
      for (int j=0; j<nCurLabel; ++j) {
        if (pnTable[j] == i) {
          pnTable[j]-=diff;
          ++cnt;
        }
      }
      if (cnt == 0) ++diff;//そのラベルはない！！
    }
    _nLabelNum = nCurLabel-diff;
    for (int i=0; i<_nLabelNum; ++i) {
      _anLabelSize[i]=0;
    }
    //二回目ラベリング
    pCur = _aUnits;
    while (pCur != pEnd) {
      if( pCur->nLabel != -1) {
//        dout << pCur->nLabel << "/" << pnTable[pCur->nLabel] << endl;
        int temp = pCur->nLabel;
        pCur->nLabel = pnTable[temp];
        ++_anLabelSize[pCur->nLabel];
      }
      ++pCur;
    }
    delete[] pnTable;
//    dout << "label:" << mt.elapsed() << endl;
    mt.restart();
    return _nLabelNum;
  }

  const int* GetLabelSize() { return _anLabelSize; }

  int GetLabelNum() const {return _nLabelNum;}
  int GetTotalUnitNum() const {return _nTotalUnitNum;}

protected:

  T* _aUnits;        //ボクセル・グリッドの本体
  int _nTotalUnitNum;//Unitが何個あるか
  int* _anSpaceSize; //領域の大きさ
  int _nLabelNum;    //ラベリングされた領域の数
  int* _anLabelSize; //各ラベルに属する要素の数

private:

};

template <class T>
inline void SimpleVote(T& Unit, int nVoteNum) {
  Unit.nVotedNum += nVoteNum;
}