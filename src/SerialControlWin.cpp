#include "StdAfx_MOTracking.h"
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include "SerialControl.h"
#include "StreamException.h"
#include "mmtimer.h"

using namespace std;


CSerialControlWin::~CSerialControlWin() {

  Close();
}

void CSerialControlWin::Close() {

  _bIsOpened = false;
  if (_hComm && (_hComm != INVALID_HANDLE_VALUE) ) {
    CloseHandle(_hComm);
  }
}

bool CSerialControlWin::Open(int comno, eBaudRate baud, bool bUSB)
{ 
  char comname[40];
  wsprintf( comname, TEXT("\\\\.\\COM%d"), comno );
  return Open(std::string(comname), baud);
}

bool CSerialControlWin::Open(const std::string &rsPortName, eBaudRate baud)
{
  _sCOMName = rsPortName;

  // シリアルポートを開ける
  _hComm = CreateFile(
    rsPortName.c_str(),         /* シリアルポートの文字列 */
    GENERIC_READ | GENERIC_WRITE, /* アクセスモード：読み書き */
    0,                /* 共有モード：他からはアクセス不可 */
    NULL,             /* セキュリティ属性：ハンドル継承せず */
    OPEN_EXISTING,          /* 作成フラグ： */
    0,
    NULL              /* テンプレートのハンドル： */
    );

  if (_hComm == INVALID_HANDLE_VALUE) {
    ESerialException e;
    e << "Failed to Open Serial Port! : " << rsPortName << " ErrorNo: " << GetLastError();
;
    throw e;
    return false;
  }

  //タイムアウトを設定
  COMMTIMEOUTS ct;
  ct.ReadIntervalTimeout = MAXDWORD;
  ct.ReadTotalTimeoutMultiplier = 0;
  ct.ReadTotalTimeoutConstant = 0;
  ct.WriteTotalTimeoutMultiplier = 0;
  ct.WriteTotalTimeoutConstant = 1000;

  if (!SetCommTimeouts(_hComm,&ct)){
    ESerialException e;
    e << "Failed to Set Timeout";
    throw e;
    return false;
  }

  // 通信属性を設定する
  DCB dcb;
  GetCommState(_hComm, &dcb); /* DCB を取得 */
  dcb.BaudRate = _aBaudRates[BR9600];
  dcb.ByteSize = 8;     // バイトサイズ
  dcb.Parity = NOPARITY;    // パリティ(使用しない,他にはEVENPARITY,ODDPARITY)
  dcb.fParity = FALSE;    // パリティを使用するか
  dcb.StopBits = ONESTOPBIT;  // ストップビット
  dcb.fOutxCtsFlow = FALSE; // 送信時に、CTS を監視するかどうか
  dcb.fOutxDsrFlow = FALSE; // 送信時に、DSR を監視するかどうか
  dcb.fDsrSensitivity = FALSE;// DSR がOFFの間は受信データを無視するか
  dcb.fDtrControl = DTR_CONTROL_DISABLE; // DTR有効/無効：　無効なら　DTR_CONTROL_DISABLE;ISABLE
  dcb.fRtsControl = RTS_CONTROL_DISABLE; // RTS制御：　RTS制御をしない場合はRTS_CONTROL_DISABLEを指定
  if (!SetCommState(_hComm, &dcb)) {
    ESerialException e;
    e << "Failed to Change DCB";
    throw e;
    return false;
  }

  _nTimeOutBefore = 0;

  ChangeBaud(baud);
  _bIsOpened = true;
  return true;
}

bool CSerialControlWin::ChangeBaud(eBaudRate baud) {

  DCB dcb;
  GetCommState(_hComm, &dcb); // DCB を取得
  dcb.BaudRate = _aBaudRates[baud];   // baudrate
  if (!SetCommState(_hComm, &dcb)) {
    ESerialException e;
    e << "Failed to set Baudrate: " << baud;
    throw e;
  }
  return true;
}

bool CSerialControlWin::WriteData(const unsigned char *buff, unsigned int data_size)
{
  DWORD dwWritten; /* ポートへ書き込んだバイト数 */
  WriteFile(_hComm, buff, data_size, &dwWritten, NULL);

  //ShowMsg(buff, data_size);

  if (dwWritten!=data_size) {
    ESerialException e;
    e << "Failed to Write! Wrong Written Bytes: expected: " << data_size << " in fact: " << dwWritten << " " << _sCOMName;
    throw e;
  }
  return true;
}

bool CSerialControlWin::Flush()
{
  DWORD dwErrors;  /* エラー情報 */
  COMSTAT ComStat; /* デバイスの状態 */
  ClearCommError(_hComm, &dwErrors, &ComStat);
//  PurgeComm(_hComm, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
  PurgeComm(_hComm, PURGE_RXCLEAR);
  ClearCommError(_hComm, &dwErrors, &ComStat);
  return true;
}

int CSerialControlWin::ReadData(unsigned char *buff, unsigned int max_size, double timeout)
{

  size_t filled = 0;
  size_t readable_size = 0;

  do {
    DWORD dwErrors;
    COMMTIMEOUTS pcto;
    GetCommTimeouts(_hComm, &pcto);
    pcto.ReadIntervalTimeout = 0;
    pcto.ReadTotalTimeoutMultiplier = 0;
    pcto.ReadTotalTimeoutConstant = 0;
    SetCommTimeouts(_hComm, &pcto);
    COMSTAT ComStat;
    ClearCommError(_hComm, &dwErrors, &ComStat);
    readable_size = (size_t)ComStat.cbInQue;
    size_t read_n = (readable_size > max_size) ? max_size : readable_size;
    DWORD n;
    if (!ReadFile(_hComm, &buff[filled], read_n, &n, NULL)) {
      ESerialException e;
      e << "Falied to Read! Cannot Read " << _sCOMName;
      throw e;
    }
    filled += n;
    readable_size -= n;

    if (filled >= max_size) {
      return filled;
    }
  } while (readable_size > 0);

  if (timeout > 0) {
    int nTimeOut = (int)round(timeout*1000);
    COMMTIMEOUTS pcto;
    GetCommTimeouts(_hComm, &pcto);
    pcto.ReadIntervalTimeout = 0;
    pcto.ReadTotalTimeoutMultiplier = 0;
    pcto.ReadTotalTimeoutConstant = nTimeOut;
    SetCommTimeouts(_hComm, &pcto);
    _nTimeOutBefore = nTimeOut;
    DWORD n;
    DWORD nTotal = 0;
    while (1) {
      ReadFile(_hComm, &buff[filled], 1, &n, NULL);
      nTotal+=n;
      if (n < 1) {
        return filled;
      }
      filled += n;
      if (filled >= max_size) {
        return filled;
      }
    }
  }
  
  return filled;
}

int CSerialControlWin::ReadUntilDelim(unsigned char* buff, unsigned int nMax, unsigned char cDelim, double dTimeOut) {

  if (dTimeOut <= 0) {
    ESStreamException e; e << "CSerialControlWin::ReadUntilDelim TimeOut Error: " << dTimeOut;
    throw e;
  }
  DWORD n;
  DWORD nTotal = 0;
  while (1) {
    int nTimeOut = (int)round(dTimeOut*1000);
    COMMTIMEOUTS pcto;
    GetCommTimeouts(_hComm, &pcto);
    pcto.ReadIntervalTimeout = 0;
    pcto.ReadTotalTimeoutMultiplier = 0;
    pcto.ReadTotalTimeoutConstant = nTimeOut;
    SetCommTimeouts(_hComm, &pcto);
    ReadFile(_hComm, &buff[nTotal], 1, &n, NULL);
    if (n < 1) {
      return (int)nTotal;
    }
    else if (buff[nTotal] == cDelim) {
      return nTotal+1;
    }
    nTotal += n;
    if (nTotal >= nMax) {
      return nTotal;
    }
  }
  return nTotal;
}
