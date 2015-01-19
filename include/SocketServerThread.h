#pragma once

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/utility.hpp>
#include <iostream>

#include "RequestQueue.h"
#include "Socket.h"

class CSocketServerThreadLoop {
public:

  CSocketServerThreadLoop(SOCKET socket, CRequestQueue* pReqQueue) {
    _pReqQueue = pReqQueue;
    _socket = socket;
  }
  virtual ~CSocketServerThreadLoop() {
  }
  void operator() ();
  int SendMessage(const std::string sMsg) {
    return send(_socket, sMsg.c_str(), (int)sMsg.size(), 0);
  }

protected:
  SOCKET _socket;
  CRequestQueue* _pReqQueue;
  char buf[4096];

  virtual void SendCommand(const std::string& rsCommand, std::istringstream& sRequest) = 0;

};

class CSocketServerThread : public boost::noncopyable
{
public:
  CSocketServerThread();
  virtual ~CSocketServerThread(void);
  bool SetupSocket(int port); //ソケット作成
  void Exit();
  void operator()();

  void SetRequestQueue(CRequestQueue* p) {_pReqQueue=p;}

protected:

  CSocketServer* _pSocket;
  volatile bool _isRunning;
  volatile bool _isWaitingConnection;
  int _port;

  CRequestQueue* _pReqQueue;

  std::vector<boost::shared_ptr<boost::thread> >  _vThreads;
  std::vector<boost::shared_ptr<CSocketServerThreadLoop> > _vSocketLoop;

  virtual boost::shared_ptr<CSocketServerThreadLoop> MakeSocketServer(SOCKET *so, CRequestQueue* pReqQueue) = 0;

};

