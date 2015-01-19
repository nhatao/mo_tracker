#include "StdAfx_MOTracking.h"
#include "SerialControl.h"
#include <iostream>
#include <stdlib.h>
#include "StreamException.h"
#include "mmtimer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <string.h>

using namespace std;



CSerialControlUnix::CSerialControlUnix() : CSerialControl() {

  _pSigSet = new sigset_t;
  _pTermIOData = new termios;

  _fd = -1;
  sigemptyset(_pSigSet);
  sigaddset(_pSigSet, SIGINT);
//    SetConfig(defaultPortName,defaultBaud,defaultParity,defaultStopBits);
}

CSerialControlUnix::~CSerialControlUnix() {

  Close();

  delete _pSigSet;
  delete _pTermIOData;
}

void CSerialControlUnix::Close() {

  _bIsOpened = false;
  if (_fd >= 0) close(_fd);
  _fd = -1;
}

bool CSerialControlUnix::Open(const std::string &rsPortName, eBaudRate baud) {

  cout << "opening " << rsPortName << endl;
  _sCOMName = rsPortName;
  _fd = open(rsPortName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (_fd >= 0) {
    cout << "Open OK" << endl;
    _bIsOpened = true;

    bzero(_pTermIOData, sizeof(termios));
    _pTermIOData->c_iflag = IGNBRK | IGNPAR;
    _pTermIOData->c_oflag = 0;
    _pTermIOData->c_lflag = 0;
    cfsetspeed(_pTermIOData, _aBaudRates[(int)baud]);
    _pTermIOData->c_cflag |= (CLOCAL | CREAD);

    //パリティなし ワンストップビット
//    _pTermIOData->c_cflag |= CSTOPB;
    _pTermIOData->c_cflag |= CS8;
    tcsetattr(_fd, TCSAFLUSH, _pTermIOData);

    _pTermIOData->c_cc[VMIN] = 0;
    _pTermIOData->c_cc[VTIME] = 0;
    tcsetattr(_fd, TCSANOW, _pTermIOData);
    ChangeBaud(baud);
    cout << "Serial Open Success: "<< rsPortName << endl;
    return true;
  }
  cout << "OpenNG!" << endl;

  return false;
}

bool CSerialControlUnix::Open(int comno, eBaudRate baud, bool bUSB)
{
  char comname[40];
  if (bUSB) {
    sprintf( comname, "/dev/ttyUSB%d", comno );
  }
  else {
    sprintf( comname, "/dev/ttyACM%d", comno );
  }
  return Open(std::string(comname), baud);
}

bool CSerialControlUnix::ChangeBaud(eBaudRate baud) {

  if (_fd >= 0) {
    cfsetspeed(_pTermIOData, _aBaudRates[(int)baud]);
    tcsetattr(_fd, TCSAFLUSH, _pTermIOData);

    return true;
  }
  return false;
}

bool CSerialControlUnix::Flush()
{
  tcflush(_fd, TCIFLUSH);
  return true;
}



bool CSerialControlUnix::WriteData(const unsigned char *buff, unsigned int data_size)
{
//  cout << "Write " << data_size << "Bytes" << endl;
  tcflush(_fd, TCIOFLUSH);
//  cout << "flash done " << endl;
  size_t nRes = (size_t)write(_fd, buff, data_size);
//  cout << "Write done " << endl;

  if (nRes != data_size) {
    ESerialException e;
    e << "Failed to Write! Wrong Written Bytes: expected: " << data_size << " in fact: " << nRes << _sCOMName;
    throw e;
  }
  return true;
}


int64_t GetTime()
{
  timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t) tv.tv_sec * 1000 + (int64_t) tv.tv_usec / 1000;
}


int CSerialControlUnix::ReadData(unsigned char *rdata, unsigned int rcount, double timeout)
{
  size_t nReadBytes = 0;


  if (timeout > 0) {
  
    int64_t start_time = GetTime();
    int64_t stop_time = start_time + (int64_t)(timeout*1000);
    int n = 0;
    while (true) {
      fd_set rfds, efds;
      FD_ZERO(&rfds);
      FD_SET(_fd, &rfds);
      FD_ZERO(&efds);
      FD_SET(_fd, &efds);

      int64_t delay = stop_time - GetTime();
      if (delay < 0) delay = 0;
      struct timeval tv;
      tv.tv_usec = (delay % 1000) * 1000;
      tv.tv_sec = delay / 1000;
      int retval = ::select(_fd + 1, &rfds, 0, &efds, &tv);
      if (retval < 0) {
        if (errno == EINTR)
          continue;
        break;
      }
      else if (!retval) {
        // TIMEOUT
        break;
      }
      int bytes = read(_fd, rdata + nReadBytes, 1);
      if (bytes < 0) {
        if (errno == EINTR) continue;
        else break;
      }
      else nReadBytes += bytes;

  //    cout << "read " << bytes << "bytes, Total=" << nReadBytes << endl;
      if (bytes==0) break;
      else {
        if (nReadBytes >= rcount) break;
      
      }
      ++n;
    }
    return nReadBytes;
  }
  else {

    int bytes = read(_fd, rdata, rcount-1);
    if (bytes < 0) {
      if (errno != EINTR) throw std::logic_error("ReadData Error!");
      else return 0;
    }
    return bytes;
  }
}


int CSerialControlUnix::ReadUntilDelim(unsigned char *rdata, unsigned int rcount, unsigned char cDelim, double timeout)
{

  if (timeout <= 0) {
    ESStreamException e; e << "CSerialControlUnix::ReadUntilDelim TimeOut Error: " << timeout << " Name: " <<  _sCOMName;
    throw e;
  }


  size_t nReadBytes = 0;  
  int64_t start_time = GetTime();
  int64_t stop_time = start_time + (int64_t)(timeout*1000);
  int n = 0;
  while (true) {

    fd_set rfds, efds;
    FD_ZERO(&rfds);
    FD_SET(_fd, &rfds);
    FD_ZERO(&efds);
    FD_SET(_fd, &efds);

    int64_t delay = stop_time - GetTime();
    if (delay < 0) delay = 0;
    struct timeval tv;
    tv.tv_usec = (delay % 1000) * 1000;
    tv.tv_sec = delay / 1000;
    int retval = ::select(_fd + 1, &rfds, 0, &efds, &tv);
    if (retval < 0) {
      if (errno == EINTR)
        continue;
      break;
    }
    else if (!retval) {
      break;
    }

    int bytes = read(_fd, rdata + nReadBytes, 1);
    if (bytes < 0) {
      if (errno == EINTR) continue;
      else break;
    }
    if (bytes==0) break; //no data
    else {

      nReadBytes += bytes;
      if ( rdata[nReadBytes-1] == cDelim ) break;
      if (nReadBytes >= rcount) break;
    }
    ++n;
  }
  
  return nReadBytes;
}
