#include "StdAfx_MOTracking.h"
#include "TrackCandidate.h"
#include "LRFData.h"
#include "Segmentation.h"
#include "OutputDebugStream.h"
#include "CLine.h"
#include "MOClassificationSVM.h"

using namespace std;

std::vector<double> CTrackCandidate::s_vSizeThr;


CDividedTrackCandidate::CDividedTrackCandidate(boost::shared_ptr<const CTrackCandidate> pOriginal, size_t nVPosBegin, size_t nVPosEnd) : CTrackCandidate() {

  _pOriginal = pOriginal;


  _pSegment = _pOriginal->GetSegment();
  _LRFGlobalCo = _pOriginal->GetLRFGlobalCo();
  _dAngleRange = _pOriginal->GetAngleRange();

  BoostVec vTotal = boost::numeric::ublas::zero_vector<double>(2);

  auto itBegin = _pOriginal->GetData().begin() + nVPosBegin;
  auto itEnd   = _pOriginal->GetData().begin() + nVPosEnd;
  for (auto it = itBegin; it != itEnd; ++it) {
    _vData.push_back(*it);
    vTotal += (*it);
  }
  vTotal /= (nVPosEnd-nVPosBegin);
  CopyArrayData();

  
  _vInitGlobal(0) = vTotal(0);
  _vInitGlobal(1) = vTotal(1);
  _LRFGlobalCo.InverseTransformVector(vTotal);
  _vInitLocal(0) = vTotal(0);
  _vInitLocal(1) = vTotal(1);

  auto itLast = itEnd-1;
  double dLen = _hypot((*itBegin)(0)-(*itLast)(0), (*itBegin)(1)-(*itLast)(1));
  _nSizeType = GetSizeType2(dLen);

  MakeTargetAngle();
  SetVelocity(_pOriginal->GetLocalVels());

  _nRegion1 = _pOriginal->GetRegion1();
  _nRegion2 = _pOriginal->GetRegion2();
  _nGroupNo = _pOriginal->GetGroupNo();

  /*
  cout << "GCo: " << _LRFGlobalCo.GetStr() << endl;
  cout << "InitLocal: " << _vInitLocal << " InitGlobal: " << _vInitGlobal << " ";
  cout << "TargetAngle : " << (_TargetAngle.dCenter-_dAngleRange).get_deg() << ":";
  cout << _TargetAngle.dCenter.get_deg() << ":" << (_TargetAngle.dCenter+_dAngleRange).get_deg() << endl;
  */

  /*
  dout << "<Origin>" << endl;
  dout << "  InitLocal: " << _pOriginal->GetLocalVec() << " InitGlobal: " << _pOriginal->GetGlobalVec() << " Seglen: " << _pOriginal->_pSegment->GetSegLen() << " SizeType: " << _pOriginal->GetSizeType() << endl;
  dout << "<Current>" << endl;
  dout << "  InitLocal: " << GetLocalVec() << " InitGlobal: " << GetGlobalVec() << " Seglen: " << dLen << " SizeType: " << GetSizeType() << endl;
  */

}

CTrackCandidate::~CTrackCandidate() {

  delete[] _aData;

};



CTrackCandidate::CTrackCandidate() {

  Init1();
}

void CTrackCandidate::CopyArrayData() {

  delete[] _aData;
  _nPointNum2 = _vData.size();
  _aData = new double [_nPointNum2*2];
  
  for (size_t i=0; i<_nPointNum2; ++i) {
    _aData[i*2] = _vData[i](0);
    _aData[i*2+1] = _vData[i](1);
  }


}


CTrackCandidate::CTrackCandidate(const CSegment *pSeg, const CCoordinates2D *pLRFGlobalCo){

  Init1();
  _pSegment = pSeg;
  _LRFGlobalCo = *pLRFGlobalCo;
//  _dAngleRange = Deg2Rad(85);
  _nSizeType = GetSizeType2(pSeg->GetSegTotalLen());

  for (CLRFData::const_iterator it = pSeg->begin(); it != pSeg->end(); ++it) {
    _vData.push_back(BoostVec(2));
    BoostVec &rV = _vData.back();
    rV(0)=(*it)(0); rV(1)=(*it)(1);
    pLRFGlobalCo->TransformVector(rV);
  }
  CopyArrayData();
  BoostVec v = pSeg->GetAveragetVec();
  _vInitLocal(0)=v(0); _vInitLocal(1)=v(1); 
  _LRFGlobalCo.TransformVector(v);
  _vInitGlobal(0)=v(0); _vInitGlobal(1)=v(1);
  SetZeroVelocity();
  MakeTargetAngle();

  if (pSeg->GetSegPointNum() != 0) {

    CAngle d1 = pSeg->begin()->GetPolar().GetAngle();
    auto it1 = pSeg->end(); --it1;
    CAngle d2 = it1->GetPolar().GetAngle();

    double dLen1 = pSeg->begin()->GetPolar().GetDist();

//    cout << "hoge: " << (d2-d1).get_deg() << endl;

    _dSideLen = abs(dLen1*tan(d2-d1));
    
  }
  else {

  }

  
}

void CTrackCandidate::Init1() {

  if (s_vSizeThr.empty()) {
    s_vSizeThr.push_back(250);
    s_vSizeThr.push_back(800);
    s_vSizeThr.push_back(1500);
  }

  _nID = -1;
  _dAngleRange = Deg2Rad(110);
  _pSegment = NULL;
  _nRegion1 = -1;
  _nRegion2 = -1;
  _nGroupNo = -1;
  _nCandidateType = 0;
  _nDbgCode = 0;

  _vInitLocal = boost::numeric::ublas::zero_vector<double>(4);
  _vInitGlobal = boost::numeric::ublas::zero_vector<double>(4);

  _aData = NULL;
  _nPointNum2 = 0;

  _dSideLen = 0;
}

void CTrackCandidate::Copy(const CTrackCandidate& rTarget) {

  _pSegment = rTarget._pSegment;
  _LRFGlobalCo = rTarget._LRFGlobalCo;
  _dAngleRange = rTarget._dAngleRange;
  _nRegion1 = rTarget._nRegion1;
  _nRegion2 = rTarget._nRegion2;

  _vData = rTarget._vData;
  CopyArrayData();
  //あとで変更する必要あり
  _vInitLocal  = rTarget._vInitLocal;
  _vInitGlobal = rTarget._vInitGlobal;
  _vLocalVels = rTarget._vLocalVels;
  _vGlobalVels = rTarget._vGlobalVels;
  _nCandidateType = rTarget._nCandidateType;
  _nSizeType = rTarget._nSizeType;

  _dSideLen = rTarget._dSideLen;

}


void CTrackCandidate::SetZeroVelocity() {
  _vLocalVels.clear();
  _vGlobalVels.clear();
  _vInitLocal(2) = 0; _vInitLocal(3) = 0;
  _vInitGlobal(2) = 0; _vInitGlobal(3) = 0;
  _vLocalVels.push_back(make_pair(0,0));
  _vGlobalVels.push_back(make_pair(0,0));
}

void CTrackCandidate::SetVelocity(const std::vector<std::pair<double, double> > &rvLocalVel,bool bAdd) {

  if (bAdd) {
    _vLocalVels.clear();
    _vGlobalVels.clear();
  }
  if (rvLocalVel.empty()) {
    SetZeroVelocity();
  }
  else {
    double dCoAngle = _LRFGlobalCo.GetRotation();
    _vLocalVels = rvLocalVel;
    for (size_t i=0; i<rvLocalVel.size(); ++i) {
      double vLen = _hypot(rvLocalVel[i].second, rvLocalVel[i].first);
      double dAngle = atan2(rvLocalVel[i].second, rvLocalVel[i].first);
      _vGlobalVels.push_back(make_pair(vLen*cos(dAngle+dCoAngle), vLen*sin(dAngle+dCoAngle)));
    }
  }

  _vInitLocal(2) = _vLocalVels[0].first; _vInitLocal(3) = _vLocalVels[0].second; 
  _vInitGlobal(2) = _vGlobalVels[0].first; _vInitGlobal(3) = _vGlobalVels[0].second;
}


void CTrackCandidate::GetInitVec(BoostVec &vResult, double dRad) const {

  vResult.resize(5);
  BoostVec vC(2);
  vC(0) = _vInitLocal(0);
  vC(1) = _vInitLocal(1);
  CAngle dAng = atan2(vC(1), vC(0));
  double dPos = _hypot(vC(0), vC(1));
  vC(0) = (dPos+dRad)*cos(dAng);
  vC(1) = (dPos+dRad)*sin(dAng);
  _LRFGlobalCo.TransformVector(vC);

  vResult(0) = vC(0);
  vResult(1) = vC(1);
  vResult(2) = _vInitGlobal(2);
  vResult(3) = _vInitGlobal(3);
  vResult(4) = dRad;
}
/*
void CTrackCandidate::SetLocalVec(double x, double y, double vx, double vy) {

  _vInitLocal(0)=x;   _vInitLocal(1)=y;
  _vInitLocal(2)=vx;  _vInitLocal(3)=vy;

  BoostVec v(2);
  v(0) = x; v(1) = y;
  _LRFGlobalCo.TransformVector(v);
  _vInitGlobal(0)=v(0); _vInitGlobal(1)=v(1);

  CAngle dCoAngle = _LRFGlobalCo.GetRotation();
  double vLen = _hypot(_vInitLocal(2), _vInitLocal(3));
  CAngle dAngle( atan2(_vInitLocal(3), _vInitLocal(2)));
  dAngle+=dCoAngle;
  _vInitGlobal(2) = vLen*cos(dAngle);
  _vInitGlobal(3) = vLen*sin(dAngle);

}
*/

size_t CTrackCandidate::GetSizeType() const {
  return _nSizeType;
}


void CTrackCandidate::MakeTargetAngle() {

  CAngle dCenter (atan2(_vInitLocal(1), _vInitLocal(0)));
//  dCenter += _LRFGlobalCo.GetRotation();
//  dCenter -= M_PI;
  dCenter += _LRFGlobalCo.GetRotation();
  dCenter -= M_PI;
  _TargetAngle.dCenter = dCenter;
  _TargetAngle.c1 = cos(dCenter-_dAngleRange);
  _TargetAngle.s1 = sin(dCenter+_dAngleRange);
  _TargetAngle.c2 = cos(dCenter-_dAngleRange);
  _TargetAngle.s2 = sin(dCenter+_dAngleRange);
  _TargetAngle.dAngleRange = _dAngleRange;

}


#include "FastMath.h"


//Medium以上のサイズの候補が一個以上あったら追跡開始
void CTrackCandidateGroup::SetInitialStatus() {

  if (nStatus == eUnknown) {
    nStatus = eNewTrack;
  }

  /*
  if (nStatus == eUnknown) {
    bool bOK = false;
    for (size_t i=0; i<vpTrackCandidates.size(); ++i) {
      boost::shared_ptr<const CTrackCandidate> &rCandi = vpTrackCandidates[i];
      if (rCandi->GetSizeType() != 0) {
        nStatus = eNewTrack;
        return;
      }
    }
    nStatus = eFalsePositive;
  }
  */
}

std::ostream &operator << (std::ostream &ro, const CTrackCandidate& r) {

  ro << r._LRFGlobalCo << " ";
  ro << r._vInitLocal << " ";
  ro << r._vInitGlobal << " ";
  ro << r._nCandidateType << " ";
  ro << r._vData.size() << " ";
  for (auto itV = r._vData.begin(); itV != r._vData.end(); ++itV) {
    ro << (*itV) << " ";
  }
  ro << r._vLocalVels.size() << " ";
  for (auto itV2 = r._vLocalVels.begin(); itV2 != r._vLocalVels.end(); ++itV2) {
    ro << (*itV2).first << " " << (*itV2).second << " ";
  }
  return ro;
}


/*
  for (auto itTrack = _vpMOTrackers.begin(); itTrack != _vpMOTrackers.end(); ++itTrack, ++i) {

    int nID = (*itTrack)->GetTrackerID();
    auto& itM = _mLogs[nID];
    if (!itM) {
      stringstream sFileName;
      sFileName << _sLogDir << "/Tracker" << nID << ".dat";
      itM = boost::shared_ptr<std::ofstream>(new std::ofstream(sFileName.str().c_str()));
    }
    (*itM) << _nFrame << " ";
    const std::list<CJPDAMOTracker::PFType> &rlTracker = (*itTrack)->GetComponents();
    (*itM) << rlTracker.size() << " ";
    for (auto it = rlTracker.begin(); it != rlTracker.end(); ++it) {
      const CJPDAMOFilter* pTracker = (const CJPDAMOFilter*)(it->get());
      const BoostVec &rResult = pTracker->GetResult();
      (*itM) << rResult << " " << pTracker->IsFormerlyMoved() << " ";
    }
    
    auto & rCandidates = _vTrackCandidateGroup2[i];
    (*itM) << rCandidates.GetTrackCandidates().size() << " ";
    for (auto itCandi = rCandidates.GetTrackCandidates().begin(); itCandi != rCandidates.GetTrackCandidates().end(); ++itCandi) {
      (*itM) << **itCandi << " ";
    }
    (*itM) << endl;
    vCurIDs.push_back(nID);
  }

68 1 [5](11716.1,48676.5,-509.399,-1025.34,163.376) 0 1 [2](10387.460452,45836.612514) [2,2]((0.316546,-0.948577),(0.948577,0.316546)) [4](3009.52,-340.894,-172.793,-529.543) [4](11663.5,48583.5,447.615,-331.533) 0 
  
  
  51 [2](11959.9,48432.5) [2](11948.5,48439.4) [2](11937.1,48446.2) [2](11922.2,48446.9) [2](11904.8,48443.2) [2](11893.4,48449.8) [2](11882,48456.3) [2](11872,48465.4) [2](11862.5,48475.3) [2](11851.4,48482.6) [2](11839.4,48488.1) [2](11827.8,48494.4) [2](11815.2,48498.9) [2](11800.3,48498.9) [2](11784.5,48497.1) [2](11767.4,48492.5) [2](11754.4,48495.9) [2](11735.5,48487.5) [2](11723.9,48493.4) [2](11712.3,48499.2) [2](11712.6,48529.2) [2](11700.9,48534.9) [2](11689.1,48540.6) [2](11675.1,48541.8) [2](11663.3,48547.4) [2](11650.6,48551.1) [2](11639.6,48558.4) [2](11634.7,48579.3) [2](11627.7,48595.7) [2](11619.7,48610.2) [2](11607.6,48615.6) [2](11591.5,48611.7) [2](11579.3,48616.9) [2](11563.7,48613.8) [2](11550,48615.2) [2](11537.1,48618.4) [2](11523.5,48619.7) [2](11511.3,48624.6) [2](11499.1,48629.5) [2](11489.1,48639.9) [2](11484.5,48664.3) [2](11501.5,48745.6) [2](11491.9,48758.9) [2](11479.2,48763.7) [2](11466.4,48768.4) [2](11453.6,48773.1) [2](11445.8,48791.8) [2](11437.6,48809.6) [2](11424.6,48814.2) [2](11411.6,48818.7) [2](11380.9,48771) 2 -172.793 -529.543 -1401.09 457.091  
  */

std::istream &operator >> (std::istream &ri, CTrackCandidate& r) {

  ri >> r._LRFGlobalCo;
  ri >> r._vInitLocal;
  ri >> r._vInitGlobal;
  ri >> r._nCandidateType;
  size_t n1;
  ri >> n1;
  BoostVec v1;
  for (size_t i=0; i<n1; ++i) {
    ri >> v1;
    r._vData.push_back(v1);
  }
  ri >> n1;
  double d1, d2;
  std::vector<std::pair<double, double> > vTemp;
  for (size_t i=0; i<n1; ++i) {
    ri >> d1 >> d2;
    vTemp.push_back(make_pair(d1, d2));
  }
  r.SetVelocity(vTemp);
  r.MakeTargetAngle();

  double dLen = 0;
  if (!r._vData.empty()) {
    dLen = _hypot( r._vData.front()(0)-r._vData.back()(0), r._vData.front()(1)-r._vData.back()(1) );
//    cout << "Len: " << dLen << endl;
  }
  r._nSizeType = r.GetSizeType2(dLen);

  return ri;
}


CTrackCandidateGroup::CTrackCandidateGroup() {
  nStatus = eUnknown;
  nRegion1 = -1;
  nRegion2 = -1;
  _pSVMProperty = boost::shared_ptr<CMOSVMProperty>(new CMOSVMProperty());
}

void CTrackCandidateGroup::Merge(CTrackCandidateGroup &rTarget, bool bClearTarget) {

  if (nStatus == eMerged) {
    ESStreamException es;
    es << "CTrackCandidateGroup Already Merged";
    throw es;
  }
  vpTrackCandidates.insert(vpTrackCandidates.end(),rTarget.vpTrackCandidates.begin(), rTarget.vpTrackCandidates.end());
  if (bClearTarget) rTarget.vpTrackCandidates.clear();
  rTarget.nStatus = eMerged;

  int n=0;
  for (auto itvp = vpTrackCandidates.begin(); itvp != vpTrackCandidates.end(); ++itvp, ++n) {
    CTrackCandidate* p = (CTrackCandidate*)itvp->get();
    p->SetID(n);
  }

}

void CTrackCandidateGroup::AddCandidate(boost::shared_ptr<CTrackCandidate> &pCandidate) {

  int nR1 = pCandidate->GetRegion1();
  int nR2 = pCandidate->GetRegion2();
  AddCandidate(pCandidate, nR1, nR2);
}

void CTrackCandidateGroup::AddCandidate(boost::shared_ptr<CTrackCandidate> &pCandidate, int nR1, int nR2) {

  pCandidate->SetID((int)vpTrackCandidates.size());
  vpTrackCandidates.push_back(pCandidate);
  if (nR1 > 0) nRegion1 = nR1;
  if (nR2 > 0) nRegion2 = nR2;
}

//CJPDAMOTracker::UpdateTrackerProb用．同じIDは入れない
void CTrackCandidateGroup::AddCandidate2(boost::shared_ptr<const CTrackCandidate> &pCandidate) {

  int nID =  pCandidate->GetID();
  for (auto it = vpTrackCandidates.begin(); it != vpTrackCandidates.end(); ++it) {
    if ((*it)->GetID() == nID) {
      return;
    }
  }

  vpTrackCandidates.push_back(pCandidate);
  int nR1 = pCandidate->GetRegion1();
  int nR2 = pCandidate->GetRegion2();
  if (nR1 > 0) nRegion1 = nR1;
  if (nR2 > 0) nRegion2 = nR2;
}


void CTrackCandidateGroup::UpdateSVMData() {

  _pSVMProperty->ConstructProperty(vpTrackCandidates);
}


std::pair<double, double> CTrackCandidateGroup::GetMinPos() const {
  return std::make_pair(_pSVMProperty->GetXMin(), _pSVMProperty->GetYMin());
}
std::pair<double, double> CTrackCandidateGroup::GetMaxPos() const {
  return std::make_pair(_pSVMProperty->GetXMax(), _pSVMProperty->GetYMax());
}
double CTrackCandidateGroup::GetXMax() const {
  return _pSVMProperty->GetXMax();
}
double CTrackCandidateGroup::GetXMin() const {
  return _pSVMProperty->GetXMin();
}
double CTrackCandidateGroup::GetYMax() const {
  return _pSVMProperty->GetYMax();
}
double CTrackCandidateGroup::GetYMin() const {
  return _pSVMProperty->GetYMin();
}
const std::vector<BoostVec> &CTrackCandidateGroup::GetBoundingBox() const {
  return _pSVMProperty->GetBBInfo().vBBPos;
}
double CTrackCandidateGroup::GetBBAngle() const {
  return _pSVMProperty->GetBBInfo().dBBAngle.get();
}

bool CTrackCandidate::IsSmall() const {

  if (GetSizeType() == 0) return true;
  else {

  }

  return false;


}

bool CTrackCandidateGroup::IsOnlySmallObjects() const {



  for (auto it =vpTrackCandidates.begin(); it != vpTrackCandidates.end(); ++it) {

    if (!(*it)->IsSmall()) return false;

  }

//  cout << "hoge Small only!" << endl;

  return true;
}




bool IsSameOne(int nRegion1, int nRegion2) {
  if ((nRegion1 >= 0) && (nRegion2 >= 0) && (nRegion1 == nRegion2)) return true;
  else return false;

}

bool CTrackCandidate::ContainSameRegion(const CTrackCandidate* pTarget) const {

  if (!pTarget->IsCombined()) {

    int nRegion11 = this->GetRegion1();
    int nRegion21 = this->GetRegion2();
    int nRegion12 = pTarget->GetRegion1();
    int nRegion22 = pTarget->GetRegion2();
    if (IsSameOne(nRegion11, nRegion12)) return true;
    if (IsSameOne(nRegion21, nRegion22)) return true;
//    if ((nRegion11 >= 0) && (nRegion12 >= 0) && (nRegion11 == nRegion12)) return true;
//    if ((nRegion21 >= 0) && (nRegion22 >= 0) && (nRegion21 == nRegion22)) return true;
    return false;
  }

  else {


    return false;
  }
}

bool CCombinedTrackCandidate::ContainSameRegion(const CTrackCandidate* pTarget) const {


  if (!pTarget->IsCombined()) {


  }

  return false;


}






/*
void CTrackCandidateGroup::MakeBB() {
  vBoundingBox.clear();

  //重心計算
  BoostVec vC(2); vC(0) = 0; vC(1) = 0;
  size_t nPointNum = 0;
  for (auto it = GetTrackCandidates().begin(); it != GetTrackCandidates().end(); ++it) {
    const vector<BoostVec> &rvPos = (*it)->GetData();
    for (auto it2 = rvPos.begin(); it2 != rvPos.end(); ++it2) {
      vC+=(*it2);
    }
    nPointNum += rvPos.size();
  }
  vC/=nPointNum;

  CCoordinates2D Co1;
  Co1.SetPos(vC(0), vC(1));
  BoostVec vRel(2);

  dBBDist = DBL_MAX;  std::vector<BoostVec> vBBPoints(4);
  for (size_t i=0; i<4; ++i) {
    vBBPoints[i].resize(2);
  }

  for (double dAng = -45.0; dAng < 45.0; dAng += 1.0) {
    double dXMin = DBL_MAX;
    double dXMax = -DBL_MAX;
    double dYMin = DBL_MAX;
    double dYMax = -DBL_MAX;
    Co1.SetRotation(Deg2Rad(dAng));

    for (auto it = GetTrackCandidates().begin(); it != GetTrackCandidates().end(); ++it) {
      const vector<BoostVec> &rvPos = (*it)->GetData();
      for (auto it2 = rvPos.begin(); it2 != rvPos.end(); ++it2) {
        Co1.InverseTransformVector((*it2), vRel);
        dXMin = min(vRel(0), dXMin);
        dXMax = max(vRel(0), dXMax);
        dYMin = min(vRel(1), dYMin);
        dYMax = max(vRel(1), dYMax);
      }
    }
    vBBPoints[0](0) = dXMin; vBBPoints[0](1) = dYMin; 
    vBBPoints[1](0) = dXMax; vBBPoints[1](1) = dYMin; 
    vBBPoints[2](0) = dXMax; vBBPoints[2](1) = dYMax; 
    vBBPoints[3](0) = dXMin; vBBPoints[3](1) = dYMax; 
    for (size_t i=0; i<vBBPoints.size(); ++i) {
      Co1.TransformVector(vBBPoints[i]);
    }

    CLineWithEndPoint aLine[4];
    aLine[0].SetBoostVec(vBBPoints[0], vBBPoints[1]);
    aLine[1].SetBoostVec(vBBPoints[1], vBBPoints[2]);
    aLine[2].SetBoostVec(vBBPoints[2], vBBPoints[3]);
    aLine[3].SetBoostVec(vBBPoints[3], vBBPoints[0]);
    std::vector<double> vMinDist(4);
    double dDistTotal=0;

    for (auto it = GetTrackCandidates().begin(); it != GetTrackCandidates().end(); ++it) {
      const vector<BoostVec> &rvPos = (*it)->GetData();
      for (auto it2 = rvPos.begin(); it2 != rvPos.end(); ++it2) {
        for (size_t j=0; j<vMinDist.size(); ++j) {
          vMinDist[j] = aLine[j].GetDistanceFromPoint(*it2);
        }
        double dMin = *(std::min_element(vMinDist.begin(), vMinDist.end()));
        dDistTotal += dMin*dMin;
      }
    }
    if (dDistTotal < dBBDist) {
      dBBDist = dDistTotal;
      dBBAngle = Deg2Rad(dAng);
      vBoundingBox = vBBPoints;
    }
  }
}


/*

const std::vector<BoostVec> &CTrackCandidateGroup::GetBoundingBox() const {return _pProperty->GetBBInfo().vBBPos;}
double CTrackCandidateGroup::GetBBAngle() const {return _pProperty->GetBBInfo().dBBAngle.get();}
double CTrackCandidateGroup::GetBBDist() const {return _pProperty->GetBBInfo().dResidual;}

*/