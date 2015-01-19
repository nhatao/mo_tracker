#pragma once

#include <boost/utility.hpp>
#include <exception>
#include <string>
#include <sstream>
#include "StreamException.h"

#include <iostream>

class CSerialControl : public boost::noncopyable
{
public:

  CSerialControl(bool bUseSioCard=false) {
    if (bUseSioCard) {
//      std::cout << "Sio!"<< std::endl;
      _aBaudRates[0] = 2150;
      _aBaudRates[1] = 4301;
      _aBaudRates[2] = 8602;
      _aBaudRates[3] = 115000;
      _aBaudRates[4] = 1075;  //ここから下は多分動かない
      _aBaudRates[5] = 12903;
      _aBaudRates[6] = 25806;
    }
    else {
      _aBaudRates[0] = 9600;
      _aBaudRates[1] = 19200;
      _aBaudRates[2] = 38400;
      _aBaudRates[3] = 500000;
      _aBaudRates[4] = 4800;
      _aBaudRates[5] = 57600;
      _aBaudRates[6] = 115200;
    }
    _bIsOpened = false;
  };
  virtual ~CSerialControl() {};


  enum eBaudRate {BR9600=0, BR19200=1, BR38400=2, BR500K=3, BR4800=4, BR57600=5, BR115200=6};

  virtual bool Open(int comno, eBaudRate baud, bool bUSB=true) = 0;
  virtual bool Open(const std::string &rsPortName, eBaudRate baud) = 0;
  virtual void Close() = 0;
  virtual bool ChangeBaud(eBaudRate baud) = 0;

  virtual bool WriteData(const unsigned char *buff, unsigned int  data_size) = 0;
  //timeoutは秒単位
  virtual int ReadData(unsigned char *buff, unsigned int max_size, double timeout=0.0) = 0;
  virtual int ReadUntilDelim(unsigned char* buff, unsigned int nMax, unsigned char cDelim, double dTimeOut) = 0;

  virtual bool Flush() = 0;
  bool IsOpened() const {return _bIsOpened;}

protected:
  int _aBaudRates[7];
  bool _bIsOpened;
};

class CSerialControlDummy : public CSerialControl
{
public:

  CSerialControlDummy(bool bUseSioCard=false) : CSerialControl() {}
  virtual ~CSerialControlDummy() {};

  virtual bool Open(int comno, eBaudRate baud, bool bUSB=true) {Throw(); return false;}
  virtual bool Open(const std::string &rsPortName, eBaudRate baud) {Throw(); return false;}
  virtual void Close() {Throw();}
  virtual bool ChangeBaud(eBaudRate baud)  {Throw(); return false;}

  virtual bool WriteData(const unsigned char *buff, unsigned int data_size) {Throw(); return false;}
  virtual int ReadData(unsigned char *buff, unsigned int max_size, double timeout=0.0)  {Throw(); return 0;}
  virtual bool Flush()  {Throw(); return false;}
  virtual int ReadUntilDelim(unsigned char* buff, unsigned int nMax, unsigned char cDelim, double dTimeOut) {return 0;}

protected:

  void Throw() {
    throw std::logic_error("SerialControl not initialized");
  }
};


#ifdef _MSC_VER

class CSerialControlWin : public CSerialControl {

public:

  CSerialControlWin(bool bUseSioCard=false) : CSerialControl(bUseSioCard), _hComm(NULL) {};
  virtual ~CSerialControlWin();

  virtual bool Open(int comno, eBaudRate baud, bool bUSB=true); //windowsはbUSBは関係ない
  virtual bool Open(const std::string &rsPortName, eBaudRate baud);
  virtual void Close();
  virtual bool ChangeBaud(eBaudRate baud);

  virtual bool WriteData(const unsigned char *buff, unsigned int data_size);
  virtual int ReadData(unsigned char *buff, unsigned int max_size, double timeout= 0.0);
  virtual int ReadUntilDelim(unsigned char* buff, unsigned int nMax, unsigned char cDelim, double dTimeOut);

  virtual bool Flush();

private:

  typedef void *HANDLE;
  HANDLE _hComm;
  std::string _sCOMName;

  int _nTimeOutBefore;
};


#else 
// struct sigset_t;
struct termios;

class CSerialControlUnix : public CSerialControl {

public:

  CSerialControlUnix();
  virtual ~CSerialControlUnix();

  virtual bool Open(int comno, eBaudRate baud, bool bUSB=true);
  virtual bool Open(const std::string &rsPortName, eBaudRate baud);
  virtual void Close();
  virtual bool ChangeBaud(eBaudRate baud);

  virtual bool WriteData(const unsigned char *buff, unsigned int data_size);
  virtual int ReadData(unsigned char *buff, unsigned int max_size, double timeout);
  virtual int ReadUntilDelim(unsigned char* buff, unsigned int nMax, unsigned char cDelim, double dTimeOut);

  virtual bool Flush();

private:

  int _fd;

  sigset_t *_pSigSet;
  termios *_pTermIOData;
  std::string _sCOMName;
};
#endif

//void ShowMsg(const unsigned char* msg, size_t len);

class ESerialException : public ESStreamException {

public:
  ESerialException() {};
  virtual ~ESerialException() throw() {}
    
  ESerialException(const ESerialException& e) : ESStreamException(e) {
  };
  ESerialException& operator=(ESerialException& e) {
    if (this==&e) return *this;
    ESStreamException::operator=(e);
    return *this;
  };

};

