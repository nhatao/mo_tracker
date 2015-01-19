#include "StdAfx_MOTracking.h"
#include "socketwin.h"

#include <iostream>
#include <sstream>
#include <math.h>

using namespace std;

CSocketServerWin::CSocketServerWin(void) : CSocketServer(), _sock(NULL)
{
}

CSocketServerWin::~CSocketServerWin(void)
{
  if(_sock) {
    closesocket(_sock);
    WSACleanup();
  }
}

bool CSocketServerWin::SetupSocket(const std::string& sHostName, int port, bool bNonBlocking) {

  WSADATA wsaData;
  struct sockaddr_in addr;

  WSAStartup(MAKEWORD(2,0), &wsaData);

  _sock = socket(AF_INET, SOCK_STREAM, 0);

  if (_sock == INVALID_SOCKET) {
    ESStreamException ess;
    ess << "failed to make socket : "  << WSAGetLastError();
    throw ess;
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.S_un.S_addr = INADDR_ANY;

  //ポートが開放されたあとすぐに使えるようにする設定
  BOOL yes = true;
  setsockopt(_sock,SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));

  ::bind(_sock, (struct sockaddr *)&addr, sizeof(addr));
  listen(_sock, SOMAXCONN);

  if (bNonBlocking) { //ちゃんとチェックしてない
    u_long val=1;
    if (ioctlsocket(_sock, FIONBIO, &val) != 0) {
      ESStreamException ess;
      ess << "faiild ioctlsocket: " << WSAGetLastError();
      throw ess;
    }
  }

  return true;
}

SOCKET CSocketServerWin::WaitForClient(double dTimeout = 0) {

  fd_set mask;
  FD_ZERO(&mask);
  FD_SET(_sock, &mask);
  timeval timeout;
  int nTimeout = (int)floor(dTimeout);
  int nTimeoutMSec = (int)((dTimeout-nTimeout)*1000*1000);
  timeout.tv_sec = nTimeout;
  timeout.tv_usec = nTimeoutMSec;
  int res;
  SOCKET mainsock;

  if (dTimeout > 0) {
    res = select(0, &mask, 0, 0, &timeout);
  }
  else {
    res = select(0, &mask, 0, 0, NULL);
  }

  if (res > 0){
    if (FD_ISSET(_sock, &mask)) {
      int len = sizeof(_LastClient);
      ZeroMemory(&_LastClient, len);
      mainsock = accept(_sock, (struct sockaddr *)&_LastClient, &len);
      if (mainsock == INVALID_SOCKET) {
        return mainsock;
      }
      if (_params.isBlock) {
        if (_params.nRecvTimeout != 0) {
          //recvでタイムアウトする設定
          setsockopt(mainsock,SOL_SOCKET, SO_RCVTIMEO, (const char *)&_params.nRecvTimeout, sizeof(_params.nRecvTimeout));
        }     }
      else {
        //ブロックしない設定
        u_long val=1;
        ioctlsocket(mainsock, FIONBIO, &val);
      }
      if (_params.nSendTimeout != 0) {
        //sendでタイムアウトする設定
        setsockopt(mainsock,SOL_SOCKET, SO_SNDTIMEO, (const char *)&_params.nSendTimeout, sizeof(_params.nSendTimeout));
      }
      return mainsock;
    }
    else {
      return NULL;
    }
  }
  else {
    return NULL;
  }
}


const char* CSocketServerWin::GetHostName() const {
  return inet_ntoa(_LastClient.sin_addr);
}


CSocketClientWin::CSocketClientWin(void) : CSocketClient(), _sock(NULL) {
  _bOpened = false;
}

CSocketClientWin::~CSocketClientWin(void){

  if(_bOpened) {
    closesocket(_sock);
    WSACleanup();
  }

}

bool CSocketClientWin::SetupSocket(const std::string& sHostName, int port, bool bNonBlocking) {
  // WinSockの初期化
  WORD version = MAKEWORD(2, 0);
  WSADATA wsa;
  WSAStartup(version, &wsa);
  int Result;
  
  _sHostName = sHostName;
  _nPort = port;
  
  // ホスト情報の作成
  LPHOSTENT hostent = gethostbyname(sHostName.c_str());
  if (!hostent) {
//    cout << "gethostbyname() failed: " << sHostName << endl;
    ostringstream oss; oss << "SocketError: gethostbyname() failed " << sHostName << "/" << port;
    std::exception e(oss.str().c_str());
    throw e;
    return false;
  }
  // ソケットの作成
  _sock = socket(AF_INET, SOCK_STREAM, 0);
  if (_sock==INVALID_SOCKET) {
    ostringstream oss; oss << "SocketError: INVALID_SOCKET "  << sHostName << "/" << port;
    std::exception e(oss.str().c_str());
    throw e;
    return false;
  }

  int flag = 1;
  int ret = setsockopt( _sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag) );
  if (ret == -1) {
    ESStreamException oss; oss << "SocketError: setsockopt";
    throw oss;
  }
  // ソケットアドレスの作成
  SOCKADDR_IN sockaddr;
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);
  sockaddr.sin_addr = *((LPIN_ADDR)*hostent->h_addr_list);
  // 接続
  Result = connect(_sock, (LPSOCKADDR)&sockaddr, sizeof(SOCKADDR_IN));
  if (Result) {
    ESStreamException oss; oss << "SocketError: Cannot Connect! " << sHostName << "/" << port;
    throw oss;
    return false;
  }

  if (bNonBlocking) {
    u_long val=1;
    if (ioctlsocket(_sock, FIONBIO, &val) != 0) {
      ESStreamException ess;
      ess << "faiild ioctlsocket: " << WSAGetLastError();
      throw ess;
    }
  }

  _bOpened = true;
  return true;
}

int CSocketClientWin::ReceiveMsg(char *pBuffer, int size) {

  if (!_bOpened) {
    strcpy(pBuffer, "NotConnected");
    return 0;
  }
  else {
    int n = recv(_sock, pBuffer, sizeof(char)*size, 0);
    if (n > 0) pBuffer[n] = '\0';

    if (n < 0) {
      if (WSAGetLastError() == WSAEWOULDBLOCK) {
        // ブロックしない設定にしたとき
        strcpy(pBuffer, "NoMesssage");
      } 
      else if (WSAGetLastError() == WSAETIMEDOUT) {
        // タイムアウト
        strcpy(pBuffer, "TimeOut");
      } 
      else {
        // それ以外のエラー
        int error = WSAGetLastError();
        string a = "OtherReason: ";
        char num[100];
        itoa(error, num, 10);
        a.append(num);
        strcpy(pBuffer, a.c_str());
      }
    }
    if (n == 0) {
      //接続切断
      strcpy(pBuffer, "SocketDisconnected");
      shutdown(_sock, SD_BOTH);
      closesocket(_sock);
    }
    return n;
  }
}

int CSocketClientWin::SendMsg(const char *pMessage, int size) {

  if (!_bOpened) {
    return 0;
  }
  if (size==0) {
    size = (int)strlen(pMessage);
  }
//  int n = send(_sock, pMessage, (int)strlen(pMessage), 0);
  int n = send(_sock, pMessage, size, 0);
  return n;
}

bool CSocketP2PServerWin::SetupSocket(const std::string& sHostName, int port, bool bNonBlocking) {

  _pSockServer.reset(new CSocketServerWin());
  _pSockServer->SetupSocket(sHostName, port, bNonBlocking);
  _sock = _pSockServer->WaitForClient(_dWaitTime);
  if (_sock == NULL) {
    ESStreamException oss; oss << "SocketError: Cannot Connect! " << sHostName << "/" << port;
    throw oss;
  }
  else {
    _nPort = port;
    _sHostName = sHostName;
  } 
  return true;
}

int CSocketP2PServerWin::ReceiveMsg(char *pBuffer, int size) {


  if (!_pSockServer) {
    strcpy(pBuffer, "NotConnected");
    return 0;
  }
  else {
    int n = recv(_sock, pBuffer, sizeof(char)*size, 0);
    if (n > 0) pBuffer[n] = '\0';

    if (n < 0) {
      if (WSAGetLastError() == WSAEWOULDBLOCK) {
        // ブロックしない設定にしたとき
        strcpy(pBuffer, "NoMesssage");
      } 
      else if (WSAGetLastError() == WSAETIMEDOUT) {
        // タイムアウト
        strcpy(pBuffer, "TimeOut");
      } 
      else {
        // それ以外のエラー
        int error = WSAGetLastError();
        string a = "OtherReason: ";
        char num[100];
        itoa(error, num, 10);
        a.append(num);
        strcpy(pBuffer, a.c_str());
      }
    }
    if (n == 0) {
      //接続切断
      strcpy(pBuffer, "SocketDisconnected");
      shutdown(_sock, SD_BOTH);
      closesocket(_sock);
      _pSockServer.reset();
    }
    return n;
  }

  return 0;
}

int CSocketP2PServerWin::SendMsg(const char *pMessage, int size) {


  if (!_pSockServer) {
    return 0;
  }
  if (size==0) {
    size = (int)strlen(pMessage);
  }
  int n = send(_sock, pMessage, size, 0);
  return n;
}
