#pragma once

#include <string>
//ソケット通信のサーバを作成するクラス
#ifdef _MSC_VER
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#define SOCKET int
#endif

struct SSocketParams {  
  bool isBlock; //recvをしたときにメッセージが入るまで待つか否か
  int nRecvTimeout; //recvをしたときのタイムアウト設定(msec) 0ならデータが来るまで待つ isBlockがfalseなら無効
  int nSendTimeout; //sendをしたときのタイムアウト設定(msec) 0ならデータが来るまで待つ
};

class CSocketBase {

public:

  CSocketBase() {
    _params.isBlock = true;
    _params.nRecvTimeout = 2000;
    _params.nSendTimeout = 2000;
  }
  virtual ~CSocketBase() {}

  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false) = 0;
  //Receiveは、エラーがあった場合-1を返し、pBufferにその内容を書き込む. 接続が切断されていたときは、0を返す

  virtual const char* GetHostName () const = 0;
//  virtual int GetPort() const = 0;

  bool SetParams(const SSocketParams& params) {
    _params = params;
    return true;
  };
  virtual int SendMsg(const char *pMessage, int size=0) = 0;
  virtual int ReceiveMsg(char *pBuffer, int size) = 0;

protected:
  SSocketParams _params;

};

//複数クライアントを扱うために,SOCKETを返す機能のみしか実装していない
class CSocketServer : public CSocketBase
{
public:

  CSocketServer(void) {};
  virtual ~CSocketServer(void) {};
  virtual SOCKET WaitForClient(double dTimeout) = 0;

  virtual int SendMsg(const char *pMessage, int size=0) {
    throw std::logic_error("SendMsg not implemented");
  }
  virtual int ReceiveMsg(char *pBuffer, int size) {
    throw std::logic_error("ReceiveMsg not implemented");
  }
};

//こちらは単一クライアント用
class CSocketP2PServer : public CSocketBase
{
public:

  CSocketP2PServer(void) {};
  virtual ~CSocketP2PServer(void) {};
  virtual int SendMsg(const char *pMessage, int size=0) = 0;
  virtual int ReceiveMsg(char *pBuffer, int size) = 0;
  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false) = 0;

  void SetWaitTime(double d) {_dWaitTime = d;}
  double GetWaitTime() const {return _dWaitTime;}

protected:
  double _dWaitTime;


};


class CSocketClient : public CSocketBase
{
public:
  CSocketClient(void) {};
  virtual ~CSocketClient(void) {};
  virtual int SendMsg(const char *pMessage, int size=0) = 0;
  virtual int ReceiveMsg(char *pBuffer, int size) = 0;
  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false) = 0;

private:

};


#ifdef _MSC_VER
#include "SocketWin.h"
#else
#include "SocketUnix.h"
#endif