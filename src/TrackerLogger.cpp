#include "StdAfx_MOTracking.h"
#include "TrackerLogger.h"

using namespace std;
using namespace boost::filesystem;

CTrackerLogger::CTrackerLogger(void)
{
}

CTrackerLogger::~CTrackerLogger(void)
{
}

void CTrackerLogger::Init(const std::string &rsFileName, const boost::filesystem::path& rDir) {

  if (!boost::filesystem::exists(rDir)) {
    if (!boost::filesystem::create_directory(rDir)) {
      ostringstream oss; oss << "create_directory failed: " << absolute(rDir);
      throw std::logic_error(oss.str().c_str());
    }
  }

  auto pFilePath = rDir/rsFileName;

  cout << "Opening log file: " << pFilePath.string() << endl;
  _pFile.reset(new ofstream(pFilePath.string()));
  _pFile->setf(ios::fixed, ios::floatfield);

  if (!_pFile->is_open()) {
    ostringstream oss; oss << "file open failed: " << pFilePath.string();
    throw std::logic_error(oss.str().c_str());
  }

}

void CTrackerLogger::Update(const CSubject& rSubject, const std::string &sMsg) {

  if (sMsg != "TrackerUpdated") {
    return;
  }
  try {
    const CMOTracker &rTracker = dynamic_cast<const CMOTracker&>(rSubject);
    CTrackerResult Result;
    rTracker.GetLatestResult(Result);

    int n=0;
    for (auto it1 = Result.first->_vpObjects.begin();
              it1 != Result.first->_vpObjects.end(); ++it1, ++n) 
    {
      const auto &pStatus = (*it1)->GetStatus();

      (*_pFile) << setprecision(3);
      (*_pFile) << Result.first->_dCurrentTime << ",";
      (*_pFile) << setprecision(2);
      (*_pFile) << (*it1)->GetID() << "," << pStatus->_vPos(0) << "," << pStatus->_vPos(1) << "," << pStatus->_vVel(0) << "," << pStatus->_vVel(1);
      if ( (*it1)->GetType() == "cylinder") {
        const SCylinderMovingObjectStatus* pStatus2 = (const SCylinderMovingObjectStatus*)(pStatus.get());
        (*_pFile) << "," << pStatus2->_dRadius;
      }
      (*_pFile) << setprecision(6);
      (*_pFile) << "," << pStatus->_dExistenceRate << endl;
    }
  }
  catch (std::exception &e) {
    cout << __FUNCTION__ << " something wrong: " << e.what() << endl;
  }

}
