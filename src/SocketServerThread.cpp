#include "StdAfx_MOTracking.h"
#include "SocketServerThread.h"
#include "RequestQueue.h"
#include "SocketWin.h"
#include <iostream>
#include <sstream>

using namespace std;


volatile bool g_bSocketLoopFinished = false; //なぜかメンバ変数にするとtrueのままで変化しない

void CSocketServerThreadLoop::operator() () {
  while (!g_bSocketLoopFinished) {
    int n = recv(_socket, buf, sizeof(char)*1000, 0);
    if (n==0) { //接続切断
      break;
    }
    else if (n<0) { //タイムアウトなどのエラー
      int error = WSAGetLastError();
      if ((error != WSAETIMEDOUT) && (error != WSAEWOULDBLOCK)){
        cout << "Socket Error: " << error << endl;
        break;
      }
    }
    else { //(n>0)
      buf[n]='\0';
      string sTemp(buf);
      istringstream sAllRequest(sTemp);
      ostringstream sResult;
      string sRequestTemp;
      while (getline(sAllRequest, sRequestTemp)) {
        istringstream sRequest(sRequestTemp);
        string sFirst;
        sRequest >> sFirst;
        if (sFirst == "command") {
          if (_pReqQueue != NULL) {
            CRequest req(sRequest.str());
            _pReqQueue->SetRequest(req);
          }
        }
        else {
          cout << "send command " << sRequest.str() << endl;
          SendCommand(sFirst, sRequest);
        }
      }
    }
  }
  shutdown(_socket, SD_BOTH);
  closesocket(_socket);
}


CSocketServerThread::CSocketServerThread() {

  _pSocket = new CSocketServerWin();
  _isWaitingConnection = false;
  _isRunning = true;
  _pReqQueue = NULL;
  _port = -1;
}
  
CSocketServerThread::~CSocketServerThread(void)
{
  delete _pSocket;
}

bool CSocketServerThread::SetupSocket(int port) {
  _port = port;
  return _pSocket->SetupSocket("localhost", port);
}

void CSocketServerThread::operator () (){

  if (_port < 0) {
    cout << "Port not Initialized!!!" << endl;
    return;
  }

  g_bSocketLoopFinished = false;
  while (_isRunning) {
    SOCKET so1 = _pSocket->WaitForClient(0.5);
    if (so1 != NULL) {
      boost::shared_ptr<CSocketServerThreadLoop> pLoop = MakeSocketServer(&so1, _pReqQueue);
      _vThreads.push_back(boost::shared_ptr<boost::thread>(
        new boost::thread(boost::ref(*pLoop))));
      _vSocketLoop.push_back(pLoop);
      cout << "Socket Connected!!" << endl;
    } 
  }
  g_bSocketLoopFinished = true;
  for (size_t i=0; i<_vThreads.size(); ++i) {
    _vThreads[i]->join();
  }
}

void CSocketServerThread::Exit() {
  _isRunning = false;
}

