#pragma once

#include "Socket.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <netinet/tcp.h>


class CSocketServerUnix : public CSocketServer {
public:
  CSocketServerUnix(void);
  virtual ~CSocketServerUnix(void);

  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false);
  virtual const char* GetHostName () const;
  virtual SOCKET WaitForClient(double dTimeout);

  const sockaddr_in &GetLastClientInfo() const {return _LastClient;}

private:
  int _sock;
  sockaddr_in _LastClient;

};

class CSocketP2PServerUnix : public CSocketP2PServer
{
public:

  CSocketP2PServerUnix(void) {
    _dWaitTime = 0.5;
  };
  virtual ~CSocketP2PServerUnix(void) {};
  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false);

  virtual int ReceiveMsg(char *pBuffer, int size);
  virtual int SendMsg(const char *pMessage, int size=0);

  int GetPort() const {return _nPort;}
  const char* GetHostName() const {return _sHostName.c_str();}
private:

  boost::shared_ptr<CSocketServerUnix> _pSockServer;
  int _sock;
  std::string _sHostName;
  int _nPort;

};


class CSocketClientUnix : public CSocketClient {
public:
  CSocketClientUnix(void);
  virtual ~CSocketClientUnix(void);

  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false);
  virtual int ReceiveMsg(char *pBuffer, int size);
  virtual int SendMsg(const char *pMessage, int size=0);

  virtual const char* GetHostName() const {return _sHostName.c_str();}
  int GetPort() const {return _nPort;}

private:

  int _sock;
  int _nPort;
  std::string _sHostName;
  bool _bOpened;
};

