#include "StdAfx_MOTracking.h"
#include "SystemData.h"
#include "LRFSystem.h"
#include "LRFSource.h"
#include "Preprocess.h"
#include "Odometry.h"
#include "DebugView.h"

using namespace std;
using namespace boost;

CMainSystemData::CMainSystemData(const CLRFSystem *pSystem) : CSystemData(pSystem)
{
  SetLRFSystem(pSystem);
//  _pPreprocess = boost::shared_ptr<CPreprocess>(new CPreprocess());
}

CMainSystemData::~CMainSystemData() {
  for (size_t s=0; s<_aRawArray.size(); ++s) {
    delete[] _aRawArray[s];
  }
}

void CMainSystemData::SetLRFSystem(const CLRFSystem *pSystem) {

  if (pSystem) {
    _pSystem = pSystem;
    _nLRFNum = pSystem->GetLRFNum();
    MakeArrays();
    for (int i=0; i<_nLRFNum; ++i) {
      _aLocalLRFCoords[i] = pSystem->GetLRFCoords(i);
      _aGlobalLRFCoords[i] = pSystem->GetLRFCoords(i);
    }
  }
}

void CMainSystemData::MakeArrays() {

  _apRawLRFData.clear();
  _apRawLRFData2.clear();
  _apLocalLRFData.clear();
  _apGlobalLRFData.clear();

  _aLocalLRFCoords.clear();
  _aGlobalLRFCoords.clear();
  _abTransformedGlobal.clear();
  _abTransformedLocal.clear();
  CCoordinates2D temp;

  for (size_t s=0; s<_aRawArray.size(); ++s) {
    delete[] _aRawArray[s];
  }
//  _aRawArray.clear();
  if (_aRawArray.size()!=(size_t)_nLRFNum) _aRawArray.resize(_nLRFNum);

  for (int i=0; i<_nLRFNum; ++i) {
    _apRawLRFData.push_back(boost::shared_ptr<CLRFData>(new CLRFData));
    _apRawLRFData2.push_back(boost::shared_ptr<CLRFData>(new CLRFData));
    _apLocalLRFData.push_back(boost::shared_ptr<CLRFData>(new CLRFData));
    _apGlobalLRFData.push_back(boost::shared_ptr<CLRFData>(new CLRFData));
    _abTransformedGlobal.push_back(false);
    _abTransformedLocal.push_back(false);
    _aLocalLRFCoords.push_back(temp);
    _aGlobalLRFCoords.push_back(temp);
    _aRawArray[i] = new double [_pSystem->GetLRFSource(i)->GetElemNum()];
  }
}

void CMainSystemData::SetGlobalCoords(const CCoordinates2D& rCurrentCoords) {


  CCoordinates2D LocalCo;
  _GlobalCoords.Transformation(LocalCo, rCurrentCoords);
  _GlobalCoords = rCurrentCoords;

  for (int i=0; i<_nLRFNum; ++i) {
    _abTransformedGlobal[i] = false;
    /*
    if (_abTransformedGlobal[i]) {
      _apGlobalLRFData[i]->TransformVector(LocalCo);
    }
    */
    if (!_abTransformedLocal[i]) LocalTransform(i);
    _aGlobalLRFCoords[i] = rCurrentCoords;
    _aGlobalLRFCoords[i].TransformLocal(_aLocalLRFCoords[i]);
  }
}


void CMainSystemData::UpdateData() {

  for (int i=0; i<_nLRFNum; ++i) {
    _abTransformedGlobal[i] = false;
    _abTransformedLocal[i] = false;
    size_t n = _pSystem->GetLRFSource(i)->GetElemNum();
    memcpy(_aRawArray[i], _pSystem->ReadRawData(i), sizeof(unsigned short)*n);
    _apRawLRFData2[i]->Clear(); 
    _apRawLRFData[i]->Clear();
    _pSystem->GetLRFSource(i)->SetData(*_apRawLRFData2[i]);
  }

  g_pDebugView(0)->Clear();
  if ((_pSystem->HaveOdometry()) && (_pSystem->GetOdometry()->HasSwing())) {
    double pitch = _pSystem->GetOdometry()->GetPitch();
    double roll = _pSystem->GetOdometry()->GetRoll();
    CBothVec temp;
    CBothVec temp2;
    for (int i=0; i<_nLRFNum; ++i) {
      _apRawLRFData[i]->Clear();
      _apLocalLRFData[i]->Clear();
      _abTransformedLocal[i] = true;
      CCoordinates2D LRFCo;
      
      for (CLRFData::iterator it = _apRawLRFData2[i]->Begin(); it != _apRawLRFData2[i]->End(); ++it) {

#if 0
//        double nPos = ((it->GetPolar().GetDeg() - GetSystem()->GetLRFSource(i)->GetFirstDeg()) / GetSystem()->GetLRFSource(i)->GetReso());
        double nPos = ((it->GetPolar().GetAngle().get_deg() - GetSystem()->GetLRFSource(i)->GetFirstDeg()) / GetSystem()->GetLRFSource(i)->GetReso());
        LRFCo.SetPos( dRobotX+nPos*dxVel*dLRFScanTick, dRobotY+nPos*dyVel*dLRFScanTick);
        LRFCo.SetRotation( dRobotR+nPos*drVel*dLRFScanTick);
        LRFCo.TransformLocal(_aLocalLRFCoords[i]);
        LRFCo.TransformVector(*it, temp);
#else
        _aLocalLRFCoords[i].TransformVector(*it, temp);
#endif

#if 1
        temp(1) = (cos(pitch)*(temp(1)));
        temp(0) = (cos(roll)*(temp(0)));
#endif
        _aLocalLRFCoords[i].InverseTransformVector(temp, temp2);

#if 0
        if (_nLRFNum == 2) {
          if (i==0) { 
            if (temp[0] > 0.5) {
              _apRawLRFData[i]->PushData(temp2);
              _apLocalLRFData[i]->PushData(temp);
            }
          }
          else {
            if (temp[0] < -0.5) {
              _apRawLRFData[i]->PushData(temp2);
              _apLocalLRFData[i]->PushData(temp);
            }
          }
        }
        else {
          _apRawLRFData[i]->PushData(temp2);
          _apLocalLRFData[i]->PushData(temp);
        }
#else 
        _apRawLRFData[i]->PushData(temp2);
        _apLocalLRFData[i]->PushData(temp);
#endif
      }
    }

  }
  else {
    for (int i=0; i<_nLRFNum; ++i) {
      _apRawLRFData[i]->Copy(*_apRawLRFData2[i]);
    }
  }
  if (_nLRFNum>0) {
    _dTime = _pSystem->GetLRFSource(0)->GetCurrentScanTime();
  }
}

void CMainSystemData::LocalTransform(int nNum) const {

  _abTransformedLocal[nNum] = true;
  _apLocalLRFData[nNum]->Clear();
  _apLocalLRFData[nNum]->Copy(*_apRawLRFData[nNum]);
  _apLocalLRFData[nNum]->TransformVector(_aLocalLRFCoords[nNum]);
}

void CMainSystemData::GlobalTransform(int nNum) const {
  _abTransformedGlobal[nNum] = true;
  _apGlobalLRFData[nNum]->Clear();

  CCoordinates2D Co1;
  MakeGlobalCoord(_GlobalCoords, _aLocalLRFCoords[nNum], Co1);
  _apGlobalLRFData[nNum]->Copy(*_apRawLRFData[nNum]);
  _apGlobalLRFData[nNum]->TransformVector(Co1);
}

CMainSystemData::CMainSystemData(const CMainSystemData& rhs) : CSystemData(GetSystem()){
  _nLRFNum = 0;
  this->operator =(rhs);
}

CMainSystemData& CMainSystemData::operator= (const CMainSystemData& rhs) {

//  cout << "copy " << this << " from " << &rhs << endl;

  if (this == &rhs) return *this;
  _dTime = rhs._dTime;
  _pSystem = rhs._pSystem;
  _GlobalCoords = rhs._GlobalCoords;
  if (rhs._nLRFNum > _nLRFNum) {
    _nLRFNum = rhs._nLRFNum;
    MakeArrays();
  }
  else {
    _nLRFNum = rhs._nLRFNum;
//    MakeArrays();
  }
  for (int i=0; i<_nLRFNum; ++i) {
    _apRawLRFData[i]->Copy(*(rhs._apRawLRFData[i]));
    _aLocalLRFCoords[i] = rhs._aLocalLRFCoords[i];
    _aGlobalLRFCoords[i] = rhs._aGlobalLRFCoords[i];
    if (rhs._abTransformedLocal[i]) {
      _apLocalLRFData[i]->Copy(*(rhs._apLocalLRFData[i]));
      _abTransformedLocal[i]=true;
    }
    else {
//      cout << "hoge3 " << this << " from " << &rhs  << endl;
      _abTransformedLocal[i]=false;
    }
    if (rhs._abTransformedGlobal[i]) {
      _apGlobalLRFData[i]->Copy(*(rhs._apGlobalLRFData[i]));
      _abTransformedGlobal[i]=true;
    }
    else {
      _abTransformedGlobal[i]=false;
    }
    memcpy(_aRawArray[i], rhs.GetRawArray(i), sizeof(unsigned short)*(_pSystem->GetLRFSource(i)->GetElemNum()));
  }
  SetGlobalCoords(_GlobalCoords);

  return *this;
}

/*
CMainSystemData::CMainSystemData(const CSubSystemData& rhs) : CSystemData(GetSystem()){
  _nLRFNum = 0;
  this->operator =(rhs);
}

CMainSystemData& CMainSystemData::operator= (const CSubSystemData& rhs) {

  _nLRFNum = (int)rhs._anNums.size();
  _GlobalCoords = rhs._pParent->_GlobalCoords;
  _pSystem = rhs._pSystem;

  MakeArrays();
  for (int i=0; i<_nLRFNum; ++i) {
    _apRawLRFData[i]->Copy(*(rhs.GetRawLRFData(i)));
    _aLocalLRFCoords[i] = *(rhs.GetLocalLRFCoords(i));
    _aGlobalLRFCoords[i] = *(rhs.GetGlobalLRFCoords(i));
//    memcpy(_aRawArray[i], rhs.GetRawArray(i), sizeof(unsigned short)*(_pSystem->GetLRFSource()[i].GetElemNum()));
  }
  SetGlobalCoords(_GlobalCoords);
  return *this;
}

CSubSystemData::CSubSystemData(const CMainSystemData *pParent, int nNum) : CSystemData(pParent->GetSystem()){
  _anNums.push_back(nNum);
  _pParent = pParent;
}

CSubSystemData::CSubSystemData(const CMainSystemData *pParent, const std::vector<int>& anNums) : CSystemData(pParent->GetSystem()){
  _anNums = anNums;
  _pParent = pParent;
}
*/

std::ostream& operator<< (std::ostream &os, const CSystemData &rData) {
  os << rData.GetLRFNum() << " " << *(rData.GetGlobalRobotCoords()) << " " << rData.GetScanTime() << " ";
  for (int i=0; i<rData.GetLRFNum(); ++i) {
    os << *(rData.GetLocalLRFCoords(i)) << " ";
    os << *(rData.GetRawLRFData(i)) << " ";
  }
  return os;

}
std::istream& operator>> (std::istream &is, CMainSystemData &rData) {

  CCoordinates2D co1;
  is >> rData._nLRFNum >> co1 >> rData._dTime;
  rData.MakeArrays();
  rData.SetGlobalCoords(co1);

  for (int i=0; i<rData.GetLRFNum(); ++i) {
    CLRFData d; CCoordinates2D c;
    is >> c >> d;
    rData._apRawLRFData[i]->Copy(d);
    rData._aLocalLRFCoords[i] = c;
    rData._abTransformedGlobal[i] = false;
    rData._abTransformedLocal[i] = false;
  }
  rData.SetGlobalCoords(rData._GlobalCoords);

//  rData._pSystem = NULL;

  return is;
}
