#include "StdAfx_MOTracking.h"
#include "SlamMapSearch.h"

#include <list>
#include <iostream>
#include <math.h>    // for sqrt
#include <time.h>
#include <fstream>
#include <sstream>
#include "mmtimer.h"

#include "OutputDebugStream.h"
#include "VoxelSpace.h"

using namespace boost;
using namespace std;


double g_d1 = 1.0;
double g_d2 = 10.0;
double g_d3 = 1.0;

#ifdef _MSC_VER
extern "C" __declspec(dllexport)
#endif
void SetTemp(double d1, double d2, double d3) {
  g_d1 = d1;
  g_d2 = d2;
  g_d3 = d3;
}



CSlamMapSearch::CSlamMapSearch(const CSlamMap *pSlamMap)
{
  _pSlamMap = pSlamMap;
  _nLocationX = pSlamMap->GetXGridNum();
  _nLocationY = pSlamMap->GetYGridNum();
  _dDistThr = 250;
//  _dDistThr = 150;
  _nSearchNotOccupiedDist = 6;

  _aLocations = new location[_nLocationX*_nLocationY];
  _aEdges = new edge[_nLocationX*_nLocationY*8];

  for (int y=0; y<_nLocationY; y++) {
    for (int x=0; x<_nLocationX; x++) {
      _aLocations[x+y*_nLocationX].x = x;
      _aLocations[x+y*_nLocationX].y = y;
    }
  }

}

CSlamMapSearch::~CSlamMapSearch(void)
{
  delete[] _aLocations;
  delete[] _aEdges;
}

//dDist=実距離(mm) 
//double GetLinkWeight(double dDist, double dBairitsu = 2.0) {
double GetLinkWeight(double dDist, double dBairitsu = -1) {
  if (dBairitsu<0) dBairitsu = g_d2;
  if (dDist<=0) return 0;
  return dBairitsu*(1.0/dDist); //dBairitsu=2.0 5hex -> 0.4 10hex -> 0.2
}

/*
template <typename M, typename G>
void MakeEdge(M& weightmap, G& g, const SSearchMapState* pState, double d) {

  if (!pCurState->GetOneObject(x+1,y)->IsOccupied()) {
    CSlamMapSearch::edge_descriptor e; bool inserted;
    tie(e, inserted) = add_edge(x+y*_nLocationX , (x+1)+y*_nLocationX, g);
    if (inserted) {
      double d = GetLinkWeight(pCurState->GetOneObject(x+1,y)->dDistanceFromObs);
              weightmap[e] = d+1;
            }
        }
}
*/
//xbegin, ybegin = ロボットの現在位置
//戻り値：1=探索なしで成功、2=探索ありで成功 0=失敗
int CSlamMapSearch::Search(double xbegin, double ybegin, double xend, double yend, bool bForceSearch) {


  if (!_pSlamMap->GetGrid()->IsInBoundCarte(xend, yend)) {
    cout << "Error: Coords Out of Region!" << endl;
    return 0;
  }
  if ( (!_aSmoothedResult.empty()) &&
       (!_pSlamMap->ObstacleOnPath(&_aSmoothedResult)) &&
       (!bForceSearch)
       )
  {
    vector<location> dummy;
    vector<location>::iterator it = _aSmoothedResult.begin();
    vector<location>::iterator itNext = _aSmoothedResult.begin();
    ++itNext;
    int rx = _pSlamMap->GetGrid()->GetXPos(xbegin);
    int ry = _pSlamMap->GetGrid()->GetYPos(ybegin);
    location l; l.x=(int)rx; l.y=(int)ry;

    while (itNext != _aSmoothedResult.end()) {
      double xa = _pSlamMap->GetGrid()->GridToPosX(it->x);
      double ya = _pSlamMap->GetGrid()->GridToPosY(it->y);
      double dest = (xa-xbegin)*(xa-xbegin)+(ya-ybegin)*(ya-ybegin);
      if (   (dest < _dDistThr*_dDistThr) 
//          && (_pSlamMap->CheckObstacleGrid( _aResult[xbegin].x, _aResult[nBegin].y, _aResult[i].x, _aResult[i].y))
          && ( (itNext==_aSmoothedResult.end()) || (!_pSlamMap->CheckObstacleGrid(rx, ry, itNext->x, itNext->y))
          ))
      {
        //目標到達+最初の点(前回のロボット位置)を消す
      }
      else {
        dummy.push_back(*it);
      }
      ++it;
      ++itNext;
    }
    dummy.push_back(l);
    _aSmoothedResult=dummy;
    return 1;
  }

//これのせいでIntellisenseがおかしい
//typedef boost::property_map<mygraph_t, boost::edge_weight_t>::type WeightMap;
//  const CSlamMap::v2State& rv2CurState = _pSlamMap->GetSearchMap();
  const CGridSpaceTemplate<SSearchMapState> *pCurState = _pSlamMap->GetSearchMap();

  mygraph_t g(_nLocationX*_nLocationY);
  const double sqrt_of_2 = sqrt(2.0);
  property_map<mygraph_t, edge_weight_t>::type weightmap = get(edge_weight, g);

  mmtimer tm;
  for (int y=1; y<_nLocationY-1; y++) {
    for (int x=1; x<_nLocationX-1; x++) {
//      if (!rv2CurState[x][y].IsOccupied()) {
      if (!pCurState->GetOneObject(x,y)->IsOccupied()) {
//        if (!rv2CurState[x+1][y].IsOccupied()) {
        if (!pCurState->GetOneObject(x+1,y)->IsOccupied()) {
            edge_descriptor e; bool inserted;
            boost::tie(e, inserted) = add_edge(x+y*_nLocationX , (x+1)+y*_nLocationX, g);
            if (inserted) {
//              double d = GetLinkWeight(rv2CurState[x+1][y].dDistanceFromObs);
              double d = GetLinkWeight(pCurState->GetOneObject(x+1,y)->dDistanceFromObs);
              weightmap[e] = d+1;
            }
        }
//        if (!rv2CurState[x][y+1].IsOccupied()) {
        if (!pCurState->GetOneObject(x,y+1)->IsOccupied()) {
            edge_descriptor e; bool inserted;
            boost::tie(e, inserted) = add_edge(x+y*_nLocationX , x+(y+1)*_nLocationX, g);
            if (inserted) {
//              double d = GetLinkWeight(rv2CurState[x][y+1].dDistanceFromObs);
              double d = GetLinkWeight(pCurState->GetOneObject(x,y+1)->dDistanceFromObs);
              weightmap[e] = d+1;
            }
        }
//        if (!rv2CurState[x+1][y-1].IsOccupied()) {
        if (!pCurState->GetOneObject(x+1,y-1)->IsOccupied()) {
            edge_descriptor e; bool inserted;
            boost::tie(e, inserted) = add_edge(x+y*_nLocationX , (x+1)+(y-1)*_nLocationX, g);
            if (inserted) {
//              double d = GetLinkWeight(rv2CurState[x+1][y-1].dDistanceFromObs);
              double d = GetLinkWeight(pCurState->GetOneObject(x+1,y-1)->dDistanceFromObs);
              weightmap[e] = d+sqrt_of_2;
            }
        }
//        if (!rv2CurState[x+1][y+1].IsOccupied()) {
        if (!pCurState->GetOneObject(x+1,y+1)->IsOccupied()) {
            edge_descriptor e; bool inserted;
            boost::tie(e, inserted) = add_edge(x+y*_nLocationX , (x+1)+(y+1)*_nLocationX, g);
            if (inserted) {
//              double d = GetLinkWeight(rv2CurState[x+1][y+1].dDistanceFromObs);
              double d = GetLinkWeight(pCurState->GetOneObject(x+1,y+1)->dDistanceFromObs);
              weightmap[e] = d+sqrt_of_2;
            }
        }
      }
    }
  }

  int x1 = _pSlamMap->GetGrid()->GetXPos(xbegin);
  int y1 = _pSlamMap->GetGrid()->GetYPos(ybegin);
  int x2 = _pSlamMap->GetGrid()->GetXPos(xend);
  int y2 = _pSlamMap->GetGrid()->GetYPos(yend);


//  if ( rv2CurState[x2][y2].IsOccupied() ) {
  if ( pCurState->GetOneObject(x2,y2)->IsOccupied() ) {
    //占領されていない場所を目標とする
    vector<location> vL;
    
    for (int dist = 1; dist < _nSearchNotOccupiedDist; ++dist) {
      vL.clear();
      //隣接=4 2マス=8 3マス=12 ...
      for (int j=0; j<dist; ++j) {
        int sa1 = j;
        int sa2 = (dist-j);
        int xx, yy;
        
        xx = x2+sa1; yy = y2+sa2;
        if ( (xx>=0) && (yy>=0) && (xx<_pSlamMap->GetGrid()->GetXGridNum()) && (yy<_pSlamMap->GetGrid()->GetYGridNum())
//          && (!rv2CurState[xx][yy].IsOccupied()) ) {
          && (!pCurState->GetOneObject(xx,yy)->IsOccupied()) ) {
          location l1 = {xx, yy};
          vL.push_back(l1);
        }
        xx = x2-sa1; yy = y2-sa2;
        if ( (xx>=0) && (yy>=0) && (xx<_pSlamMap->GetGrid()->GetXGridNum()) && (yy<_pSlamMap->GetGrid()->GetYGridNum())
//          && (!rv2CurState[xx][yy].IsOccupied()) ) {
          && (!pCurState->GetOneObject(xx,yy)->IsOccupied()) ) {
          location l1 = {xx, yy};
          vL.push_back(l1);
        }
        xx = x2+sa2; yy = y2-sa1;
        if ( (xx>=0) && (yy>=0) && (xx<_pSlamMap->GetGrid()->GetXGridNum()) && (yy<_pSlamMap->GetGrid()->GetYGridNum())
//          && (!rv2CurState[xx][yy].IsOccupied()) ) {
          && (!pCurState->GetOneObject(xx,yy)->IsOccupied()) ) {
          location l1 = {xx, yy};
          vL.push_back(l1);
        }
        xx = x2-sa2; yy = y2+sa1;
        if ( (xx>=0) && (yy>=0) && (xx<_pSlamMap->GetGrid()->GetXGridNum()) && (yy<_pSlamMap->GetGrid()->GetYGridNum())
//          && (!rv2CurState[xx][yy].IsOccupied()) ) {
          && (!pCurState->GetOneObject(xx,yy)->IsOccupied()) ) {
          location l1 = {xx, yy};
          vL.push_back(l1);
        }
      }
      if (!vL.empty() ) {
        location lnearest; double dnearest = DBL_MAX;
        for (vector<location>::iterator it=vL.begin(); it != vL.end(); ++it) {
          double d = (it->x-x1)*(it->x-x1)+(it->y-y1)*(it->y-y1);
          if (d < dnearest) {
            lnearest = *it; 
            dnearest = d;
          }
        }
        x2 = lnearest.x; y2 = lnearest.y;
        goto label1; //候補が見つかったので処理終了
      }
    }
    //候補が見つからなかった
    cout << "no kouho" << endl;
    //_aResult.clear();
    //_aSmoothedResult.clear();
    return 0;
  }
label1:
  vertex start = x1 + y1  *_nLocationX;
  vertex goal =  x2 + y2  *_nLocationX;

//  dout << start << " " << goal << endl;  
//  dout << "Start vertex: "<< (_aLocations[start]).x << " " << (_aLocations[start]).y << endl;
 // dout << "Goal vertex: " << (_aLocations[goal]).x << " " << (_aLocations[goal]).y << endl;
  tm.restart();

  vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
  vector<cost> d(num_vertices(g));
  try {
    // call astar named parameter interface
    astar_search
      (g, start,
       distance_heuristic<mygraph_t, cost, location*  >
        (_aLocations, goal),
       predecessor_map(&p[0]).distance_map(&d[0]).
       visitor(astar_goal_visitor<vertex>(goal)));
    
  } catch(found_goal) { // found a path to the goal

    _aResult.clear();
    vertex v = goal;
    while (p[v] != v) {
      _aResult.push_back(_aLocations[v]);
      v = p[v];
    }
    if ((_aResult.back().x!=x1) || (_aResult.back().y!=y1)) {
      location lStart;
      lStart.x=x1; lStart.y=y1;
      _aResult.push_back(lStart);
    }

//    SmoothPath(rv2CurState);
    SmoothPath(pCurState);
    return 2;
  }

  _aResult.clear();
  _aSmoothedResult.clear();
  dout << "Didn't find a path" << endl;
  cout << "Didn't find a path" << endl;

  return 0;
}

double GetDistance(int x1, int y1, int x2, int y2) {
  return sqrt((double)((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}
static double g_dDistanceSum;
void GetWeightedDistanceSum(const SSearchMapState &rUnit) {
//  double d = GetLinkWeight(rUnit.dDistanceFromObs, 0.5);
  double d = GetLinkWeight(rUnit.dDistanceFromObs, g_d3);
  g_dDistanceSum += d;
}

#include <boost/ref.hpp>

//void CSlamMapSearch::SmoothPath(const CSlamMap::v2State& rv2CurState) {
void CSlamMapSearch::SmoothPath(const CGridSpaceTemplate<SSearchMapState> * pCurState) {

  size_t nResultSize = _aResult.size();
  _aSmoothedResult.clear();

  if (_pSlamMap->ObstacleOnPath(&_aResult)) {
    dout << "BUG!! Obstacle On Path Before Smoothing" << endl;
    cout << "BUG!! Obstacle On Path Before Smoothing" << endl;
  }
  _aSmoothedResult.push_back(_aResult[0]);

  size_t nBegin=0;
  for (size_t i=2; i<nResultSize; ++i) {
    if (_pSlamMap->CheckObstacleGrid( _aResult[nBegin].x, _aResult[nBegin].y, _aResult[i].x, _aResult[i].y)) 
    {
      nBegin = i-1;
      _aSmoothedResult.push_back(_aResult[nBegin]);
    }
    else {
      /*
      cout << "(" << _aResult[nBegin].x << "," << _aResult[nBegin].y <<") -> ";
      cout << "(" << _aResult[i-1].x << "," << _aResult[i-1].y <<") -> ";
      cout << "(" << _aResult[i].x << "," << _aResult[i].y <<")" << endl;
      */

      g_dDistanceSum=0;
//      boost::function1<void, const SSearchMapState&, std::allocator<boost::function_base> > f = CSlamMap::SearchMapFunc(&GetWeightedDistanceSum);
      boost::function1<void, const SSearchMapState& > f = CSlamMap::SearchMapFunc(&GetWeightedDistanceSum);
      _pSlamMap->ApplyFuncOnPath( _aResult[nBegin].x,  _aResult[nBegin].y,  _aResult[i].x,  _aResult[i].y, f);
      double dSumSmooth = g_dDistanceSum
                          + GetDistance(_aResult[nBegin].x, _aResult[nBegin].y, _aResult[i].x, _aResult[i].y);
//      cout << "SumSmooth:" << dSumSmooth << " (" << g_dDistanceSum << "+" << GetDistance(_aResult[nBegin].x, _aResult[nBegin].y, _aResult[i].x, _aResult[i].y) << ")" << endl;
      g_dDistanceSum=0;
      //      _pSlamMap->ApplyFuncOnPath(_aResult[nBegin].x, _aResult[nBegin].y, _aResult[i-1].x, _aResult[i-1].y, CSlamMap::SearchMapFunc(&GetWeightedDistanceSum));     
//      boost::function1<void, const SSearchMapState&, std::allocator<boost::function_base> > f2 = CSlamMap::SearchMapFunc(&GetWeightedDistanceSum);
      boost::function1<void, const SSearchMapState&> f2 = CSlamMap::SearchMapFunc(&GetWeightedDistanceSum);
      _pSlamMap->ApplyFuncOnPath(_aResult[nBegin].x, _aResult[nBegin].y, _aResult[i-1].x, _aResult[i-1].y, f2);
//      double d1 = max(0.0, GetLinkWeight(rv2CurState[_aResult[i].x][_aResult[i].y].dDistanceFromObs));
      double d1 = max(0.0, GetLinkWeight(pCurState->GetOneObject(_aResult[i].x,_aResult[i].y)->dDistanceFromObs));
      double d2 = GetDistance(_aResult[nBegin].x, _aResult[nBegin].y, _aResult[i-1].x, _aResult[i-1].y);
      double d3 = GetDistance(_aResult[i].x, _aResult[i].y, _aResult[i-1].x, _aResult[i-1].y);
      double dSumNonSmooth = g_dDistanceSum + d1 + d2 + d3;
//      cout << "SumNonSmooth:" << dSumNonSmooth << " (" << g_dDistanceSum << "+" << d1 << "+" << d2 << "+" << d3 << ")" << endl;

      if (dSumSmooth > dSumNonSmooth) {
        nBegin = i-1;
        _aSmoothedResult.push_back(_aResult[nBegin]);
      }
    }
  }

  _aSmoothedResult.push_back(_aResult[nResultSize-1]);

  if (_pSlamMap->ObstacleOnPath(&_aSmoothedResult)) {
    dout << "BUG!! Obstacle On Path After Smoothing" << endl;
    cout << "BUG!! Obstacle On Path After Smoothing" << endl;
  }

}

/*
void CSlamMapSearch::SmoothPath(const CSlamMap::v2State& rv2CurState) {

  size_t nResultSize = _aResult.size();
  _aSmoothedResult.clear();

  if (_pSlamMap->ObstacleOnPath(&_aResult)) {
    dout << "BUG!! Obstacle On Path Before Smoothing" << endl;
    cout << "BUG!! Obstacle On Path Before Smoothing" << endl;
  }

  std::vector<location> aSmoothedResult1;
  std::vector<location> aSmoothedResult2;

  aSmoothedResult1.push_back(_aResult[0]);
  for (size_t i=0; i<(nResultSize-2); ++i) {
    if ( ((_aResult[i].x)-(_aResult[i+1].x)) == ((_aResult[i+1].x)-(_aResult[i+2].x)) &&
         ((_aResult[i].y)-(_aResult[i+1].y)) == ((_aResult[i+1].y)-(_aResult[i+2].y)) ) 
    {
      //do nothing
    }
    else {
      aSmoothedResult1.push_back(_aResult[i+1]);
    }
  }
  aSmoothedResult1.push_back(_aResult[nResultSize-1]);

  std::vector<location> *pCurrent = &aSmoothedResult1;
  std::vector<location> *pNext = &aSmoothedResult2;
  nResultSize = pCurrent->size();
  
  if (nResultSize == 2) { 
    _aSmoothedResult=aSmoothedResult1;
    return;   
  }

  if (_pSlamMap->ObstacleOnPath(&aSmoothedResult1)) {
    dout << "BUG!! Obstacle On Path In Smoothing" << endl;
    cout << "BUG!! Obstacle On Path In Smoothing" << endl;
  }

  for (int j=0; j<20; ++j) {
    nResultSize = pCurrent->size();
    if (nResultSize == 2) {
      break;
    }
    size_t nMinRand = rand()%(nResultSize-2);
    size_t nMaxRand = rand()%(nResultSize-nMinRand-2)+nMinRand+2;
    bool b = _pSlamMap->CheckObstacleGrid((*pCurrent)[nMinRand].x, (*pCurrent)[nMinRand].y, (*pCurrent)[nMaxRand].x, (*pCurrent)[nMaxRand].y);

    if (!b) {
      for (size_t i=0; i<nResultSize; ++i) {
        if ((i<=nMinRand) || (i>=nMaxRand)) {
          pNext->push_back(pCurrent->at(i));
        }
      }
      std::vector<location> *pTemp = pNext;
      pNext = pCurrent;
      pCurrent = pTemp;
      pNext->clear();
    }
    else {
      pNext->clear();
    }
  }



  _aSmoothedResult = (*pCurrent);

  if (_pSlamMap->ObstacleOnPath(&_aSmoothedResult)) {
    dout << "BUG!! Obstacle On Path After Smoothing" << endl;
    cout << "BUG!! Obstacle On Path After Smoothing" << endl;
  }

}
*/

/*
  for (int j=0; j<10; ++j) {

  if (nResultSize == 3) {
//      bool b1 = CheckObstacle(rv2CurState, (*pCurrent)[0].x, (*pCurrent)[0].y, (*pCurrent)[2].x, (*pCurrent)[2].y);
      bool b1 = _pSlamMap->CheckObstacleGrid((*pCurrent)[0].x, (*pCurrent)[0].y, (*pCurrent)[2].x, (*pCurrent)[2].y);
      _aSmoothedResult.push_back((*pCurrent)[0]);
      if (b1) {
        _aSmoothedResult.push_back((*pCurrent)[1]); 
      }
      _aSmoothedResult.push_back((*pCurrent)[2]);
      return;
    }

    int nRand1 = rand()%(nResultSize-2)+1; //0と最後を避ける
    int nRand2 = rand()%(nResultSize-3)+1; //0と最後とnRand1を避ける
    if (nRand2 >= nRand1) {
      ++nRand2;
    }
    int nMinRand = min(nRand1, nRand2);
    int nMaxRand = max(nRand1, nRand2);

//    bool b1 = CheckObstacle(rv2CurState, _aResult[0].x, _aResult[0].y, _aResult[nMinRand].x, _aResult[nMinRand].y);
//    bool b2 = CheckObstacle(rv2CurState, _aResult[nMinRand].x, _aResult[nMinRand].y, _aResult[nMaxRand].x, _aResult[nMaxRand].y);
//    bool b3 = CheckObstacle(rv2CurState, _aResult[nMaxRand].x, _aResult[nMaxRand].y, _aResult[nResultSize-1].x, _aResult[nResultSize-1].y);

    bool b1 = _pSlamMap->CheckObstacleGrid((*pCurrent)[0].x, (*pCurrent)[0].y, (*pCurrent)[nMinRand].x, (*pCurrent)[nMinRand].y);
    bool b2 = _pSlamMap->CheckObstacleGrid((*pCurrent)[nMinRand].x, (*pCurrent)[nMinRand].y, (*pCurrent)[nMaxRand].x, (*pCurrent)[nMaxRand].y);
    bool b3 = _pSlamMap->CheckObstacleGrid((*pCurrent)[nMaxRand].x, (*pCurrent)[nMaxRand].y, (*pCurrent)[nResultSize-1].x, (*pCurrent)[nResultSize-1].y);

    pNext->push_back( (*pCurrent)[0] );
    if (b1) {
      for (int i=1; i<nMinRand; ++i) {
        pNext->push_back( (*pCurrent)[i] );
      }
    }
    pNext->push_back( (*pCurrent)[nMinRand] );
    if (b2) {
      for (int i=nMinRand+1; i<nMaxRand; ++i) {
        pNext->push_back( (*pCurrent)[i] );
      }
    }
//    if (nMinRand != nMaxRand) pNext->push_back( (*pCurrent)[nMaxRand] );
    pNext->push_back( (*pCurrent)[nMaxRand] );

    if (b3) {
      for (int i=nMaxRand+1; i<nResultSize; ++i) {
        pNext->push_back( (*pCurrent)[i] );
      }
    }
    pNext->push_back( (*pCurrent)[nResultSize-1] );

//    if (!b1) dout << "0-" << nMinRand << " OK!" << endl;
//    if (!b2) dout << nMinRand << "-" << nMaxRand << " OK!" << endl;
//    if (!b3) dout << nMaxRand << "-Last" <<  " OK!" << endl;

    nResultSize = (int)pNext->size();
    std::vector<location> *pTemp = pNext;
    pNext = pCurrent;
    pCurrent = pTemp;
    pNext->clear();
  }
  */






