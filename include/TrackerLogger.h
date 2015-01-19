#pragma once

#include "MOTracker.h"
#include "observer.h"
#include <boost/filesystem.hpp>

class CTrackerLogger : public CObserver
{
public:
  CTrackerLogger();
  virtual ~CTrackerLogger(void);

  void Init(const std::string &rsFileName, const boost::filesystem::path& rDir = boost::filesystem::current_path());
  virtual void Update(const CSubject& rSubject, const std::string &sMsg);

protected:

  boost::shared_ptr<std::ofstream> _pFile;
};

