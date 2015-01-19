#pragma once

#include "Socket.h"
#include <winsock2.h>
#include <boost/shared_ptr.hpp>

class CSocketServerWin :
  public CSocketServer
{
public:
  CSocketServerWin(void);
  virtual ~CSocketServerWin(void);

  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false);
  virtual const char* GetHostName () const; 
  virtual SOCKET WaitForClient(double dTimeout);

  const sockaddr_in &GetLastClientInfo() const {return _LastClient;}

private:

  SOCKET _sock;
  sockaddr_in _LastClient;
};

class CSocketP2PServerWin : public CSocketP2PServer
{
public:

  CSocketP2PServerWin(void) {
    _dWaitTime = 0.5;
  };
  virtual ~CSocketP2PServerWin(void) {};
  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false);

  virtual int ReceiveMsg(char *pBuffer, int size);
  virtual int SendMsg(const char *pMessage, int size=0);

  int GetPort() const {return _nPort;}
  const char* GetHostName() const {return _sHostName.c_str();}
private:

  boost::shared_ptr<CSocketServerWin> _pSockServer;
  SOCKET _sock;
  std::string _sHostName;
  int _nPort;

};

class CSocketClientWin :
  public CSocketClient
{
public:
  CSocketClientWin(void);
  virtual ~CSocketClientWin(void);

  virtual bool SetupSocket(const std::string& sHostName, int port, bool bNonBlocking = false);
  virtual int ReceiveMsg(char *pBuffer, int size);
  virtual int SendMsg(const char *pMessage, int size=0);

  virtual const char* GetHostName() const {return _sHostName.c_str();}
  int GetPort() const {return _nPort;}

private:

  SOCKET _sock;
  int _nPort;
  std::string _sHostName;
  bool _bOpened;

  fd_set _fds;

};

