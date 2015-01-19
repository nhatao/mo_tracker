#include "StdAfx_MOTracking.h"
#include "SocketUnix.h"

#include <iostream>
#include <sstream>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>


using namespace std;

#if 1

CSocketServerUnix::CSocketServerUnix(void) : CSocketServer()
{
  _sock = 0;
}

CSocketServerUnix::~CSocketServerUnix(void)
{
  if(_sock) {
    close(_sock);
  }
}

bool CSocketServerUnix::SetupSocket(const std::string& sHostName, int port, bool bNonBlocking) {

  sockaddr_in addr;

  _sock = socket(AF_INET, SOCK_STREAM, 0);
  if (_sock < 0) {
    cout << "failed to make socket!" << endl;
    return false;
  }

  if (bNonBlocking) {
    struct timeval socket_timeout;
    socket_timeout.tv_sec  = 5;
    socket_timeout.tv_usec = 0;
    int ret;
    if ((ret=setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&socket_timeout, sizeof(socket_timeout)))!=0){
      perror(NULL);
      return false;
    }
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;

  //ポートが開放されたあとすぐに使えるようにする設定
  int yes = 1;
  setsockopt(_sock,SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));

  ::bind(_sock, (struct sockaddr *)&addr, sizeof(addr));
  listen(_sock, SOMAXCONN);


  /*
  if (bNonBlocking) {
    int val = 1;
    ioctl(_sock, FIONBIO, &val);
  }
  else {
    int val = 0;
    ioctl(_sock, FIONBIO, &val);
  }
  */


  return true;
}

/*
sock0 = socket(AF_INET, SOCK_STREAM, 0);
 if (sock0 < 0) {
   perror("socket");
   return 1;
 }

 addr.sin_family = AF_INET;
 addr.sin_port = htons(12345);
 addr.sin_addr.s_addr = INADDR_ANY;

 if (bind(sock0, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
   perror("bind");
   return 1;
 }

 if (listen(sock0, 5) != 0) {
   perror("listen");
   return 1;
 }

 while (1) {
   len = sizeof(client);
   sock = accept(sock0, (struct sockaddr *)&client, &len);
   if (sock < 0) {
     perror("accept");
     break;
   }

   n = write(sock, "HELLO", 5);
   if (n < 1) {
     perror("write");
     break;
   }

   close(sock);
 }

 close(sock0);

 return 0;
}
*/

SOCKET CSocketServerUnix::WaitForClient(double dTimeout) {


  sockaddr_in client;
  unsigned int len = sizeof(client);
  return accept(_sock, (struct sockaddr *)&client, &len);
  //実装してない

  /*
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
      if (mainsock < 0) {
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
      cout << "hogege" << endl;
      return NULL;
    }
  }
  else {
//    cout << "timeout" << endl;
    return NULL;
  }
//  cout << "accepted connection from "<< inet_ntoa(client.sin_addr) <<  "  port=" << ntohs(client.sin_port) << endl;
*/
}


const char* CSocketServerUnix::GetHostName() const {
  return inet_ntoa(_LastClient.sin_addr);
}
#endif

#if 0
int
main() {
  /* IP アドレス、ポート番号、ソケット */
  char destination[80];
  unsigned short port = 9876;
  int dstSocket;

  /* sockaddr_in 構造体 */
  struct sockaddr_in dstAddr;

  /* 各種パラメータ */
  int status;
  int numsnt;
  char *toSendText = "This is a test";

  /************************************************************/
  /* 相手先アドレスの入力 */
  printf("Connect to ? : (name or IP address) ");
  scanf("%s", destination);

  /* sockaddr_in 構造体のセット */
  memset(&dstAddr, 0, sizeof(dstAddr));
  dstAddr.sin_port = htons(port);
  dstAddr.sin_family = AF_INET;
  stAddr.sin_addr.s_addr = inet_addr(destination);
 
  /* ソケット生成 */
  dstSocket = socket(AF_INET, SOCK_STREAM, 0);

  /* 接続 */
  printf("Trying to connect to %s: \n", destination);
  connect(dstSocket, (struct sockaddr *) &dstAddr, sizeof(dstAddr));

  /* パケット送出 */
  for(i=0; i<10; i++) {
    printf("sending...\n");
    send(dstSocket, toSendText, strlen(toSendText)+1, 0);
    sleep(1);
  }

  /* ソケット終了 */
  close(dstSocket);
}
#endif

CSocketClientUnix::CSocketClientUnix(void) : CSocketClient(), _sock(0) {
  _bOpened = false;
}

CSocketClientUnix::~CSocketClientUnix(void){

}

bool CSocketClientUnix::SetupSocket(const std::string& sHostName, int port, bool bNonBlocking) {
  int Result;
  
  _sHostName = sHostName;
  _nPort = port;
  

  /*
  // ホスト情報の作成
  LPHOSTENT hostent = gethostbyname(sHostName.c_str());
  if (!hostent) {
//    cout << "gethostbyname() failed: " << sHostName << endl;
    ostringstream oss; oss << "SocketError: gethostbyname() failed " << sHostName << "/" << port;
    std::exception e(oss.str().c_str());
    throw e;
    return false;
  }
  */
  // ソケットの作成
  _sock = socket(AF_INET, SOCK_STREAM, 0);
  if (_sock<0) {
    ESStreamException oss; oss << "SocketError: INVALID_SOCKET "  << sHostName << "/" << port << " errono: " << errno;
    throw oss;
    return false;
  }

  int flag = 1;
  int ret = setsockopt( _sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag) );
  if (ret == -1) {
    ESStreamException oss; oss << "SocketError: setsockopt";
    throw oss;
  }


  // ソケットアドレスの作成
  sockaddr_in sockaddr;
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);
//  sockaddr.sin_addr = *((LPIN_ADDR)*hostent->h_addr_list);
  sockaddr.sin_addr.s_addr = inet_addr(sHostName.c_str());
  // 接続
  Result = connect(_sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
  if (Result) {
    ESStreamException oss; oss << "SocketError: Cannot Connect! " << sHostName << "/" << port;
    throw oss;
    return false;
  }

  if (bNonBlocking) {
    int val = 1;
    if (ioctl(_sock, FIONBIO, &val) < 0) {
      ESStreamException oss; oss << "SocketError: SetNonBlocking Failed: " << sHostName << "/" << port << " Errono: " << errno;
      throw oss;
    }
  }

  _bOpened = true;
  return true;
}

int CSocketClientUnix::ReceiveMsg(char *pBuffer, int size) {


  if (!_bOpened) {
    strcpy(pBuffer, "NotConnected");
    return 0;
  }
  else {
    int n = recv(_sock, pBuffer, sizeof(char)*size, 0);
    if (n > 0) pBuffer[n] = '\0';
    else if (n < 0) {

    }

    /*
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
    */
    else if (n == 0) {
      //接続切断
      strcpy(pBuffer, "SocketDisconnected");
//      shutdown(_sock, SD_BOTH);
//      closesocket(_sock);
      close(_sock);
    }
    return n;
  }
}

int CSocketClientUnix::SendMsg(const char *pMessage, int size) {

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



bool CSocketP2PServerUnix::SetupSocket(const std::string& sHostName, int port, bool bNonBlocking) {

  if (!(_pSockServer.get()) || (port != _nPort)) {
    _pSockServer = boost::shared_ptr<CSocketServerUnix> (new CSocketServerUnix());
    _pSockServer->SetupSocket(sHostName, port, bNonBlocking);
  }
  _nPort = port;
  _sHostName = sHostName;
  _sock = _pSockServer->WaitForClient(_dWaitTime);

  if (_sock < 0) {
    ESStreamException oss; oss << "SocketError: Cannot Connect! " << sHostName << "/" << port;
    throw oss;
  }
  else {
  } 
  return true;
}


int CSocketP2PServerUnix::ReceiveMsg(char *pBuffer, int size) {


  if (!_pSockServer) {
    strcpy(pBuffer, "NotConnected");
    return 0;
  }
  else {
    int n = recv(_sock, pBuffer, sizeof(char)*size, 0);
    if (n > 0) pBuffer[n] = '\0';

    if (n < 0) {
      /*
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
      */
      strcpy(pBuffer, "SocketError");
    }
    if (n == 0) {
      //接続切断
      strcpy(pBuffer, "SocketDisconnected");
//      shutdown(_sock, SD_BOTH);
//      closesocket(_sock);
      close(_sock);     
      _pSockServer.reset();
    }
    return n;
  }

  return 0;
}

int CSocketP2PServerUnix::SendMsg(const char *pMessage, int size) {


  if (!_pSockServer) {
    return 0;
  }
  if (size==0) {
    size = (int)strlen(pMessage);
  }
  int n = send(_sock, pMessage, size, 0);
  return n;
}
