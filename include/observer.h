#pragma once
#include <string>
#include <list>
#include <boost/shared_ptr.hpp>

class CSubject;

class CObserver
{
public:
  CObserver();
  virtual ~CObserver(void);
  virtual void Update(const CSubject& rSubject, const std::string &sMsg) = 0;
  
private:
};


class CSubject
{
public:
  CSubject(void) {}
  virtual ~CSubject(void) {}
  void AddObserver(boost::shared_ptr<CObserver> pObserver);
  void RemoveObserver(boost::shared_ptr<CObserver> pObserver);

protected:
  void Notify(const std::string &sMsg);

private:
  std::list<boost::shared_ptr<CObserver> > m_lObservers;
};
