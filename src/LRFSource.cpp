#include "StdAfx_MOTracking.h"

#include "LRFSource.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include "mmtimer.h"
using namespace std;

const unsigned short *CLRFSource::ReadData() const {

  const unsigned short* pRaw = ReadDataImpl();
  int nElemNum = GetElemNum();
  pRaw+=GetFirstPos();
  if (_LRFProperty._bInverse) {
    unsigned short* pData = _aData+nElemNum-1;
    for (int i=0; i< nElemNum; ++i) {
      (*pData) = (*pRaw);
      --pData; 
      pRaw+=(1+_nInterval);
    }
  }
  else {
    unsigned short* pData = _aData;
    for (int i=0; i< nElemNum; ++i) {
      (*pData) = (*pRaw);
      ++pData; 
      pRaw+=(1+_nInterval);
    }
  }
  return _aData;
}

CLRFFromFile::CLRFFromFile(void)
{
  _pFin = NULL;
  _aCurrentRawData = NULL;
  _aData = NULL;
  _sLRFName = "SICK";
}

CLRFFromFile::~CLRFFromFile(void)
{
  delete _pFin;
}

bool CLRFFromFile::Init(const std::string& sFileName, int nElemNum, double dReso, double dFirstDeg, const std::string &rsLRFName, bool bHasTime, int nFirstLine) {

  _LRFProperty._nElemNum = nElemNum;
  _LRFProperty._dReso.set_deg(dReso);
  _LRFProperty._dFirstAngle.set_deg(dFirstDeg);
  _LRFProperty._bModeMM = true;
  _LRFProperty._bInverse = false;

  return Init(sFileName, _LRFProperty, rsLRFName, bHasTime, nFirstLine);
}

bool CLRFFromFile::Init(const std::string& sFileName, const SLRFProperty &rProperty, const std::string &rsLRFName, bool bHasTime, int nFirstLine) {

  _LRFProperty = rProperty;
  delete _pFin;    
  _pFin = new ifstream(sFileName.c_str());

  if (!(*_pFin).is_open()) {
    ESStreamException e; e << "Cannot Open File: " << sFileName;
    throw e;
    return false;
  }

  _sLRFName = rsLRFName;
  _sFileName = sFileName;
  _nFirstPos = 0;
  _nLastPos = (int)_LRFProperty._nElemNum;

  _bHasTime = bHasTime;
  _dCurTime = 0;

  if (_sLRFName == "") {
    _sLRFName = "SICK";
  }
  InitArray();

  _nFirstLine = nFirstLine;
  string temp;
  for (int i=0; i<_nFirstLine; ++i) {
    if (!getline((*_pFin), temp)) {
      ESStreamException e; e << "File Line Exceeds: " << sFileName << " Required: " << _nFirstLine << " Current: " << i;
      throw e;
    }
  }


  return true;
}

void CLRFFromFile::Reset() {
  int nf = _nFirstPos; int nb = _nLastPos;
  Init(
    _sFileName, 
    (int)_LRFProperty._nElemNum, 
    _LRFProperty._dReso.get_deg(),    
    _LRFProperty._dFirstAngle.get_deg(),
    _sLRFName, _bHasTime, _nFirstLine);
  SetPosLimit(nf, nb);
}

void CLRFFromFile::Measure() {

  string temp, elem;  
  if (getline((*_pFin), temp)) {
    stringstream ss(temp);
    for (size_t i=0; i<_LRFProperty._nElemNum; i++) {
      ss >> _aCurrentRawData[i];
      if (_sLRFName=="SICK") {
        if (_aCurrentRawData[i]<50) _aCurrentRawData[i]=0;
        if (_aCurrentRawData[i]>8180) _aCurrentRawData[i]=0;
      }
      else if (_sLRFName=="URG") {
        if (_aCurrentRawData[i]<=101) {
          _aCurrentRawData[i]=0;
        }
        else if (_aCurrentRawData[i]==60000) {
          _aCurrentRawData[i]=0;
        }
      }
      else {
        if (_aCurrentRawData[i]<=101) {
          _aCurrentRawData[i]=0;
        }
      }
    }
    if (_bHasTime) {
      ss >> _dCurTime;
    }
  }
  else {
    ESStreamException e; e << "FileFinished";
    throw e;
  }
}

const unsigned short* CLRFFromFile::ReadDataImpl() const{

  return _aCurrentRawData;
}

bool CLRFFromFile::InitAsTopURG(const std::string& sFileName, bool bHasTime) {
  return Init(sFileName, 1440, 0.25, -45 -90, "URG", bHasTime);
}
bool CLRFFromFile::InitAsBlackURG(const std::string& sFileName, bool bHasTime) {
  return Init(sFileName, 439, 360.0/552.0, -(219*360.0/552), "URG", bHasTime);
}

