#pragma once
#include "NHSensorManager.h"
#include "StreamException.h"
#include <iostream>
#include <iomanip>


//CNHSensorFromFileImplを挟まないと，vectorなどに代入できない
class CNHSensorFromFileImpl :
  public INHSensor
{
public:
  CNHSensorFromFileImpl(const std::string &rsName) : INHSensor(rsName)
  {};
  virtual ~CNHSensorFromFileImpl(void) {};

  virtual void Init();
  virtual bool Measure();
  virtual unsigned int GetMinUpdateTime() const {
    return _nMinTime;
  }
  const std::string& GetSensorName() const {return _sSensorName;}
  //nStatus=0 最も近い時刻のデータ nStatus=1 その時間以前で最も近い時刻のデータ nStatus=2 その時間以降で最も近い時刻のデータ
  size_t GetNearestLine(double dTime, int nStatus=0) const;
  void MeasureNearestTime(INHSensorStatus &rStatus, double dTime, int nStatus=0) const;
  void MeasureNthStatus(INHSensorStatus &rStatus, size_t n) const;
  size_t GetStatusNum() const {return _vdTimes.size();}

  double GetLastTime() const {
    return _vdTimes.back();
  }
  const std::string& GetLoadLogDir() const {return _sLoadLogDir;}

  void GetDataforGivenPeriod(std::vector<boost::shared_ptr<INHSensorStatus> > &raResult, double dTimeBegin, double dTimeEnd);

  //debug
  const std::vector<double>& GetTimes() const {return _vdTimes;}

  protected:
  virtual bool UpdateStatusImpl(INHSensorStatus *pStatus) const;
  std::string _sFileName;
  std::string _sLoadLogDir;
  unsigned int _nMinTime;

  std::vector<double> _vdTimes;

protected:
  virtual void SetNthStatus(INHSensorStatus &rStatus, size_t nLine) const = 0;

  
};

template <class TStatus>
class CNHSensorFromFile : public CNHSensorFromFileImpl {
public:
  CNHSensorFromFile(const std::string &rsName) : CNHSensorFromFileImpl(rsName)
  {};
  virtual ~CNHSensorFromFile(void) {};
  virtual boost::shared_ptr<INHSensorStatus> MakeSensorStatus() const {
    return boost::shared_ptr<INHSensorStatus>(new TStatus(this));
  };
  bool SetFile(const std::string &rsFileName, unsigned int nMinTime);

protected:

  virtual void SetNthStatus(INHSensorStatus &rStatus, size_t nLine) const {
    TStatus &r1 = (TStatus &)rStatus;
    TStatus &r2 = (TStatus &)(*(_vpResults[nLine]));
    r1 = r2;
  }
  std::vector<boost::shared_ptr<INHSensorStatus> > _vpResults;
};

#include <fcntl.h>
#ifdef _MSC_VER
#undef min
#undef max
  #include <io.h>
  #include <sys/stat.h>
  #include <fcntl.h>
#else 
  #include <sys/io.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <fcntl.h>
#define _read read
#define _close close
#endif

template <class TStatus>
bool CNHSensorFromFile<TStatus>::SetFile(const std::string &rsFileName, unsigned int nMinTime) {
  using namespace std;
  _sFileName = rsFileName;
  _nMinTime = nMinTime;
//  _vpLineData.clear();
  _vdTimes.clear();

  int fh;
#ifdef _MSC_VER
  if( _sopen_s(&fh, _sFileName.c_str(), _O_RDWR, _SH_DENYNO, 0) != 0) {
#else
  fh = open(_sFileName.c_str(), O_RDWR, 0);
  if (fh == -1) {
#endif
    ESStreamException ess;
    ess << "File Not Found: " << _sFileName;
    throw ess;
  }

  string::size_type nLastPeriod = _sFileName.rfind( "/" );
  if (nLastPeriod == string::npos) {
    _sLoadLogDir = ".";
  }
  else {
    _sLoadLogDir = _sFileName.substr(0, nLastPeriod);
  }
  //long nFileSize = _filelength(fh);
  struct stat Stat;
  fstat(fh, &Stat);
  unsigned int nFileSize = Stat.st_size;

  unsigned int nBufMaxSize = 1024*1024*1; //1MB
  char *pBuf = new char[nBufMaxSize + 1];
  std::cout << _sFileName << " FileSize: " << nFileSize << endl;

  unsigned int nBufOffset = 0;
  unsigned int nReadTotal = 0;
  
  while (true) {

    long nReadBytes = _read(fh, pBuf+nBufOffset, std::min(nFileSize-nReadTotal, nBufMaxSize-nBufOffset));
    if (nReadBytes <= 0) {
      break;
    }
    pBuf[nBufOffset+nReadBytes] = '\0';

    nBufOffset = 0;
    nReadTotal+=nReadBytes;

    char *pFirst = pBuf;
    char *pEnd = pBuf;
    bool bFinished = false;
    while (true) {
      while(true) {
        if (*pEnd == '\n') {
          *pEnd = '\0';
          break;
        }
        if (*pEnd == '\0') {
          bFinished = true;
          break;
        }
        ++pEnd;
      }
      if (bFinished) {
        memcpy(pBuf, pFirst, pEnd-pFirst);
        nBufOffset = (pEnd-pFirst);
        break;
      }
      else if (pEnd!=pFirst) {

        istringstream iss(pFirst);
        char *p=pEnd;
        while (*p != ' ') --p;
        double dTime = atof(p);
        ++pEnd;
        pFirst = pEnd;
        auto pStatus = MakeSensorStatus();
        pStatus->ReadFromLog(iss, dTime);
        _vpResults.push_back(pStatus);
        _vdTimes.push_back(pStatus->GetMeasuredTime());
      }
      else {
        break;
      }
    }
  }
  _close(fh);
  delete[] pBuf;
  return true;
}
