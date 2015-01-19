#pragma once

#include <queue>
#include <string>

#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

class CRequest
{
public:
  CRequest(const std::string& rName) {
    _name = rName;
  };
  virtual ~CRequest() {};

  const char* GetName() const {
    return _name.c_str();
  };

private:
  std::string _name;
};


class CRequestQueue
{
public:
  
  CRequestQueue(void){};
  virtual ~CRequestQueue(void){};

  void SetRequest(CRequest &req) {
    lock lk(_mutex);
    _ReqQueue.push(req);
  }
  CRequest GetRequest() {
    lock lk(_mutex);
    while (_ReqQueue.empty()) {
      CRequest nullreq("");
      return nullreq;
    }
    CRequest temp = _ReqQueue.front();
    _ReqQueue.pop();
    return temp;
  }

  bool IsEmpty() {
    return _ReqQueue.empty();
  }
  size_t GetSize() {
    return _ReqQueue.size();
  }


private:

  typedef boost::mutex::scoped_lock lock;
  boost::mutex _mutex;
  boost::condition _waitset;

  std::queue<CRequest> _ReqQueue;
};
