#include "StdAfx_MOTracking.h"

#include "observer.h"
#include <boost/mem_fn.hpp>
#include <boost/bind.hpp>
#include <algorithm>
#include <iostream>
#include "mmtimer.h"

using namespace std;

CObserver::CObserver(){
}

CObserver::~CObserver() {
}

void CSubject::AddObserver(boost::shared_ptr<CObserver> pObserver) {
  if (m_lObservers.end() == find(m_lObservers.begin(), m_lObservers.end(), pObserver)) 
    m_lObservers.push_back(pObserver);
  else cout << "Observer Already Registered: " << pObserver << endl;
}
void CSubject::RemoveObserver(boost::shared_ptr<CObserver> pObserver) {
  m_lObservers.remove(pObserver);
}


void CSubject::Notify(const std::string &sMsg) {
  list<boost::shared_ptr<CObserver> >::iterator it = m_lObservers.begin();
  while ( it != m_lObservers.end()) {
    (*it)->Update(*this, sMsg);
    ++it;
  }
}

