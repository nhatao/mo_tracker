#pragma once

//http://nis-lab.is.s.u-tokyo.ac.jp/~yuji/Work/KDT1/KDTApplet.html
//上記のページのアプレットを参考に作成
//一瞬3Dでも動きそうに見えるが実はできない
#include "stdlib.h"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <float.h>
#include <boost/bind.hpp>
#include <iostream>
#include <boost/any.hpp>
#include <iomanip>

template <class T>
inline bool lessPNode(const T& rN1, const T& rN2, int n) {
  return (rN1->GetKeys()[n] < rN2->GetKeys()[n]);
}

template <int DIM>
class CKDTree
{
public:

  class KDTreeNode;
  typedef KDTreeNode* PNode;

  class KDTreeNode {
  public:
    explicit KDTreeNode(const double* aKeys, int nID=-1) {
      memcpy(_aKeys, aKeys, sizeof(double)*DIM);
      _nID = nID;
      _pLow = _pHigh = NULL;
    }
    ~KDTreeNode() {
      delete _pLow;
      delete _pHigh;
    }
    void SetKeys(const double* aKeys, int nID) {
      memcpy(_aKeys, aKeys, sizeof(double)*DIM);
      _nID = nID;
      _pLow = _pHigh = NULL;
    }
    const double* GetKeys() const {return _aKeys;}
    PNode GetLow() {return _pLow;}
    PNode GetHigh() {return _pHigh;}
    void SetLow(PNode p) {_pLow = p;}
    void SetHigh(PNode p) {_pHigh = p;}
    int GetID() const { return _nID;}
    void Copy (const KDTreeNode* pFrom) {
      memcpy(_aKeys, pFrom->_aKeys, sizeof(double)*DIM);
      _nID = pFrom->_nID;
      _pLow = pFrom->_pLow;
      _pHigh = pFrom->_pHigh;
    }
  private:
    double _aKeys[DIM];
    PNode _pLow;
    PNode _pHigh;
    int _nID;
  };

  enum KDState {
    LOW, HIGH, SAME
  };

  CKDTree(void) {_pRoot = PNode(NULL);};
  virtual ~CKDTree(void) {
    delete _pRoot;
  };

  void Insert(PNode pNode) {

    if (_pRoot==NULL) {
      _pRoot = pNode;
      _vNodes.push_back(_pRoot);
      return;
    }
    PNode q = _pRoot;
    int disc = 0;
    while (true) {
      KDState s = Successor(q, pNode->GetKeys(), disc);
      switch(s) {
        case LOW:
          if (q->GetLow() == NULL) {
            q->SetLow(pNode);
            _vNodes.push_back(pNode);
            return;
          }
          q = q->GetLow();
          break;

        case HIGH:
          if (q->GetHigh() == NULL) {
            q->SetHigh(pNode);
            _vNodes.push_back(pNode);
            return;
          }
          q = q->GetHigh();
          break;
        case SAME:
          return;
      }
      disc = (disc+1) % DIM;
    }
  }

  void Insert(const double* aKeys, int nID = -1) {
    Insert(PNode(new KDTreeNode(aKeys, nID)));
  }
  //最近傍点を探索
  void SearchNearest(const double *pTarget, std::pair<double, PNode> &rResult ) const {
    SearchNearest2(pTarget, rResult);
  }

  void SearchNearest2(const double *pTarget, std::pair<double, PNode> &rResult ) const {
    std::pair<PNode, PNode> pKouho[DIM];
    SearchNearest2Impl(_pRoot, pTarget, pKouho,0);
    double dMin = DBL_MAX;
    PNode pRes = NULL;
    for (int i=0; i<DIM; ++i) {
      if (pKouho[i].first) {
        double d = Distance(pTarget, pKouho[i].first->GetKeys());
        if (d < dMin) {
          dMin = d;
          pRes = pKouho[i].first;
        }
      }
      if (pKouho[i].second) {
        double d = Distance(pTarget, pKouho[i].second->GetKeys());
        if (d < dMin) {
          dMin = d;
          pRes = pKouho[i].second;
        }
      }
    }
    rResult = std::make_pair(dMin, pRes);
  }
  void SearchNearest2Impl(PNode pNode, const double *pTarget, std::pair<PNode, PNode> *pKouho, int disc) const {
    if (pNode == NULL) return;
    KDState s = Successor(pNode, pTarget, disc);
    int disc2 = (disc+1)%DIM;
    if (s == SAME) {
      pKouho[disc].first = pNode;
      pKouho[disc].second = pNode;
      return;
    }
    else if (s == LOW) {
      pKouho[disc].first = pNode;
      SearchNearest2Impl(pNode->GetLow(),  pTarget, pKouho, disc2);
    }
    else {
      pKouho[disc].second = pNode;
      SearchNearest2Impl(pNode->GetHigh(), pTarget, pKouho, disc2);
    }
  }

  //num:最近傍点を何個まで求めるか
  int Search(const double *pTarget, int num, std::vector<std::pair<double, PNode> > &raResult ) const {
    std::cout << "Obsolete Function CKDTree::Search Called" << std::endl;
    if (_vNodes.empty()) return 0;
    double pB[DIM*2];
    double pNode[DIM];
    for (int i=0; i<DIM; ++i) {
      pNode[i] = 0;
      pB[i*2]   = -DBL_MAX;
      pB[i*2+1] = DBL_MAX;
    }
    if (raResult.size() != (size_t)num) {
      raResult.clear();
      for (int i=0; i<num; ++i) {
        raResult.push_back(std::make_pair(DBL_MAX, PNode(new KDTreeNode(pNode))));
      }
    }
    else {
      for (int i=0; i<num; ++i) {
        raResult[i].first = DBL_MAX;
        raResult[i].second->SetKeys(pNode, -1);
      }
    }
    _nSearchCount = 0;
    SearchIter(_pRoot, pTarget, raResult, pB, 0);
    for (int i=0; i<num; ++i) {
      if (raResult[i].first == DBL_MAX) {
        return i;
      }
    }
    return num;
  }

  //pLowとpHighの間にある点を全て求める 同じ大きさは含める
  void Search(double dX1, double dY1, double dX2, double dY2, std::vector<const double*> &raResult ) const {
    double aLow[2]; double aHigh[2];
    aLow[0]  = std::min(dX1, dX2);
    aLow[1]  = std::min(dY1, dY2);
    aHigh[0] = std::max(dX1, dX2);
    aHigh[1] = std::max(dY1, dY2);
    SearchImpl(_pRoot, aLow, aHigh, raResult, 0);
  }
  void Search(double dX1, double dY1, double dX2, double dY2, std::vector<PNode> &raResult ) const {
    double aLow[2]; double aHigh[2];
    aLow[0]  = std::min(dX1, dX2);
    aLow[1]  = std::min(dY1, dY2);
    aHigh[0] = std::max(dX1, dX2);
    aHigh[1] = std::max(dY1, dY2);
    SearchImpl(_pRoot, aLow, aHigh, raResult, 0);
  }
  void Search(const double *pLow, const double *pHigh, std::vector<const double*> &raResult ) const {
    SearchImpl(_pRoot, pLow, pHigh, raResult, 0);
  }
  void Search(const double *pLow, const double *pHigh, std::vector<PNode> &raResult ) const {
    SearchImpl(_pRoot, pLow, pHigh, raResult, 0);
  }

  void Optimize() {
    if (_pRoot == NULL) return;
    std::sort(_vNodes.begin(), _vNodes.end(), boost::bind(&lessPNode<typename CKDTree<DIM>::PNode>, _1, _2, 0));
    typename std::vector<PNode>::iterator itMid = _vNodes.begin(); itMid+=_vNodes.size()/2;
    _pRoot = *itMid;
    _pRoot->SetLow( OptIter(_vNodes.begin(), itMid,1) );
    _pRoot->SetHigh( OptIter(itMid+1, _vNodes.end(),1) );
  }

  PNode GetRoot() const { return _pRoot;}
  size_t GetNodeNum() const {return _vNodes.size();}
  const std::vector<PNode>& GetNode() const {return _vNodes;}

  double Distance(const double* pA, const double* pB) const{
    double res = 0;
    for (int i=0; i<DIM; ++i) {
      res += Distance(pA[i], pB[i]);
    }
    return res;
  }

  void Clear() {
    delete _pRoot;
    _pRoot = PNode();
    _vNodes.clear();
  }


private:

  void SearchImpl(PNode pNode, const double *pLow, const double *pHigh, std::vector<const double*> &raResult, int disc) const{
    using namespace std;
    if (pNode == NULL) return;
    const double* pCur = pNode->GetKeys();
    bool bRes = true;
    for (size_t i=0; i<DIM; ++i) {
      if (( pLow[i] > pCur[i] ) || ( pCur[i] > pHigh[i] )) bRes = false;
    }
    int disc2 = (disc+1)%DIM;
    if (bRes) {
      raResult.push_back(pCur);
      SearchImpl(pNode->GetHigh(), pLow, pHigh, raResult, disc2);
      SearchImpl(pNode->GetLow(),  pLow, pHigh, raResult, disc2);
    }
    else {
      if ((Successor(pNode, pHigh, disc) != LOW)) {
        SearchImpl(pNode->GetHigh(), pLow, pHigh, raResult, disc2);
      }
      if ((Successor(pNode, pLow, disc) != HIGH)) {
        SearchImpl(pNode->GetLow(),  pLow, pHigh, raResult, disc2);
      }
    }
  }

  void SearchImpl(PNode pNode, const double *pLow, const double *pHigh, std::vector<PNode> &raResult, int disc) const{
    using namespace std;
    if (pNode == NULL) return;
    const double* pCur = pNode->GetKeys();
    bool bRes = true;
    for (size_t i=0; i<DIM; ++i) {
      if (( pLow[i] > pCur[i] ) || ( pCur[i] > pHigh[i] )) bRes = false;
    }
    int disc2 = (disc+1)%DIM;
    if (bRes) {
      raResult.push_back(pNode);
      SearchImpl(pNode->GetHigh(), pLow, pHigh, raResult, disc2);
      SearchImpl(pNode->GetLow(),  pLow, pHigh, raResult, disc2);
    }
    else {
      if ((Successor(pNode, pHigh, disc) != LOW)) {
        SearchImpl(pNode->GetHigh(), pLow, pHigh, raResult, disc2);
      }
      if ((Successor(pNode, pLow, disc) != HIGH)) {
        SearchImpl(pNode->GetLow(),  pLow, pHigh, raResult, disc2);
      }
    }
  }

  KDState Successor(PNode q, const double* keys,int disc) const{
    if ( q->GetKeys()[disc] > keys[disc] ) return LOW;
    else if ( q->GetKeys()[disc] < keys[disc] ) return HIGH;
    int i = (disc+1)%DIM;
    while ( i != disc ) {
      if ( q->GetKeys()[i] > keys[i] ) return LOW;
      else if ( q->GetKeys()[i] < keys[i] ) return HIGH;
      if ( ++i == DIM ) i = 0;
    }
    return SAME;
  }

  int SearchIter(PNode pQ, const double* pTarget, std::vector<std::pair<double, PNode> > &raResult, double *pB, int disc) const {
  
    if (pQ == NULL) return 0;
    double p = pQ->GetKeys()[disc];
    double temp;
    if ( pTarget[disc] < p ) {
      temp = pB[disc*2+1];
      pB[disc*2+1] = p;
      if ( SearchIter(pQ->GetLow(),pTarget,raResult,pB,(disc+1)%DIM) == -1 ) {
        return -1;
      }
      pB[disc*2+1] = temp;
    }
    else {
      temp = pB[disc*2];
      pB[disc*2] = p;
      if ( SearchIter(pQ->GetHigh(),pTarget,raResult,pB,(disc+1)%DIM) == -1 ) {
        return -1;
      }
      pB[disc*2] = temp;
    }
    SearchTest(pQ, pTarget, raResult);

    if ( pTarget[disc] < p ) {
      temp = pB[disc*2];
      pB[disc*2] = p;
      if (SearchOverlap(pTarget, pB, raResult[raResult.size()-1].first)) {
        if ( SearchIter(pQ->GetHigh(),pTarget,raResult,pB,(disc+1)%DIM) == -1 ) {       
          return -1;
        }
      }
      pB[disc*2] = temp;
    }
    else {
      temp = pB[disc*2+1];
      pB[disc*2+1] = p;
      if (SearchOverlap(pTarget, pB, raResult[raResult.size()-1].first)) {
        if ( SearchIter(pQ->GetLow(),pTarget,raResult,pB,(disc+1)%DIM) == -1 ) {
          return -1;
        }
      }
      pB[disc*2+1] = temp;
    }
    if ( SearchDone(pTarget, pB, raResult[raResult.size()-1].first) ) return -1;

    return 0;
  }

  bool SearchOverlap(const double* pTarget, double* pB, double limit) const{
    using namespace std;
    double sum = 0;
    for (int i=0; i<DIM; ++i) {
      if (pTarget[i] < pB[i*2] ) {
        sum += Distance(pTarget[i], pB[i*2]);
        if (sum > limit) {
          return false;
        }
      }
      if (pTarget[i] > pB[i*2+1] ) {
        sum += Distance(pTarget[i], pB[i*2+1]);
        if (sum > limit) {
          return false;
        }
      }
    }
    return true;
  }

  void SearchTest (PNode pKey, const double* pTarget, std::vector<std::pair<double, PNode> > &raResult) const {

    double dist = Distance(pKey->GetKeys(), pTarget);
    for (size_t i=0; i<raResult.size(); ++i) {
      if (dist < raResult[i].first) {
        for (size_t j=raResult.size()-1; j>i; j--) {
//          raResult[j].second = raResult[j-1].second;
          raResult[j].second->Copy(raResult[j-1].second);
          raResult[j].first  = raResult[j-1].first;
        }
//        raResult[i].second = pKey;
        raResult[i].second->Copy(pKey);
        raResult[i].first = dist;
        break;
      }
    }
    ++_nSearchCount;
  }
  bool SearchDone(const double* pTarget, double* pB, double limit) const {
    for (int i=0; i<DIM; ++i) {
      if ( (Distance(pTarget[i], pB[i*2]) <= limit) || (Distance(pTarget[i], pB[i*2+1]) <= limit) ) {
        return false;
      }
    }
    return true;
  }

  PNode OptIter(typename std::vector<PNode>::iterator itBegin, typename std::vector<PNode>::iterator itEnd, int disc) {
    if (itBegin==itEnd) return PNode();
    std::sort(itBegin, itEnd, boost::bind(&lessPNode<typename CKDTree<DIM>::PNode>, _1, _2, disc));
    typename std::vector<PNode>::iterator itMid = itBegin + (itEnd-itBegin)/2;
    PNode q = *itMid;
    q->SetLow(OptIter(itBegin, itMid,(disc+1)%DIM));
    q->SetHigh(OptIter(itMid+1, itEnd,(disc+1)%DIM));     
    return q;
  }

  double Distance(double a, double b) const {
    return (a-b)*(a-b);
  }

  PNode _pRoot;

  mutable int _nSearchCount;
  std::vector<PNode> _vNodes; //記憶領域
};

template <int DIM>
void ShowNode (std::ostream &os, typename CKDTree<DIM>::KDTreeNode& rNode, int nWidth = 0, int nHeight = 0) {

  if (nHeight == 1) {
    for (int i=0; i<nWidth; ++i) {
      os << " ";
    }
  }
  os << "(";
  for (int i=0; i<DIM; ++i) {
    os << " ";
    os << std::setw(4) << std::right << rNode.GetKeys()[i];
  }
  os << ")";
  if (rNode.GetHigh() != NULL) {
    os << "->";
    ShowNode<DIM>(os, *rNode.GetHigh(), nWidth+5*DIM+4, 0);
  }
  if (rNode.GetLow() != NULL) {
    os << "\n";
    ShowNode<DIM>(os, *rNode.GetLow(), nWidth, 1);
  }
}




template <int DIM>
std::ostream& operator << (std::ostream &os, CKDTree<DIM>& rTree) {

  typename CKDTree<DIM>::PNode pCur = rTree.GetRoot();
  ShowNode<2>(os, *pCur);
  return os;  
}
