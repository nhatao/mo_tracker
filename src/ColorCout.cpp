#include "StdAfx_MOTracking.h"
#include "ColorCout.h"
//#include <iostream>

#ifdef _MSC_VER
#include <windows.h>

std::streamsize ColorCout::write(const char *s, std::streamsize n){


  unsigned short nColor = _vColFG[(int)_eForeGround] | _vColBG[(int)_eBackGround];

  DWORD written = 0;
  const HANDLE handle = ::GetStdHandle(STD_OUTPUT_HANDLE);
  ::SetConsoleTextAttribute(handle, nColor);
  ::WriteConsoleA(handle, s, (DWORD)n, &written, NULL);
  const WORD init = FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE;
  ::SetConsoleTextAttribute(handle, init);
  return written;
}

ColorCout::ColorCout() {

  _vColFG.push_back(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
  _vColFG.push_back(FOREGROUND_RED   | FOREGROUND_INTENSITY);
  _vColFG.push_back(FOREGROUND_GREEN | FOREGROUND_INTENSITY);
  _vColFG.push_back(FOREGROUND_BLUE  | FOREGROUND_INTENSITY);
  _vColFG.push_back(FOREGROUND_RED   | FOREGROUND_BLUE  | FOREGROUND_INTENSITY);
  _vColFG.push_back(FOREGROUND_GREEN | FOREGROUND_BLUE  | FOREGROUND_INTENSITY);
  _vColFG.push_back(FOREGROUND_RED   | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
  _vColFG.push_back(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY);
  _vColFG.push_back(0);
  _vColFG.push_back(FOREGROUND_INTENSITY);

  _vColBG.push_back(0);
  _vColBG.push_back(BACKGROUND_RED   | BACKGROUND_INTENSITY);
  _vColBG.push_back(BACKGROUND_GREEN | BACKGROUND_INTENSITY);
  _vColBG.push_back(BACKGROUND_BLUE  | BACKGROUND_INTENSITY);
  _vColBG.push_back(BACKGROUND_RED   | BACKGROUND_BLUE  | BACKGROUND_INTENSITY);
  _vColBG.push_back(BACKGROUND_GREEN | BACKGROUND_BLUE  | BACKGROUND_INTENSITY);
  _vColBG.push_back(BACKGROUND_RED   | BACKGROUND_GREEN | BACKGROUND_INTENSITY);
  _vColBG.push_back(BACKGROUND_RED | BACKGROUND_GREEN | BACKGROUND_BLUE | BACKGROUND_INTENSITY);
  _vColBG.push_back(0);
  _vColBG.push_back(BACKGROUND_INTENSITY);

  SetColor(eDefault, eDefault);
}

#else

#include <stdio.h>

std::streamsize ColorCout::write(const char *s, std::streamsize n){

  printf("%s", _svColBG[(int)_eBackGround].c_str() );
  printf("%s", _svColFG[(int)_eForeGround].c_str() );

  for(std::streamsize i=0; i<n; ++i, ++s) {
    putchar(*s);
  }
  printf("%s", _svColBG[(int)eDefault].c_str() );
  printf("%s", _svColFG[(int)eDefault].c_str() );

  return n;
}

ColorCout::ColorCout() {

  //    eRed, eGreen, eBlue, eMagenta, eCyan, eYellow, eWhite, eBlack, eGray, eColorNum
  _svColFG.push_back("\x1b[39m"); //default
  _svColFG.push_back("\x1b[31m"); //red
  _svColFG.push_back("\x1b[32m"); //green
  _svColFG.push_back("\x1b[34m"); //blue
  _svColFG.push_back("\x1b[35m"); //magenta
  _svColFG.push_back("\x1b[36m"); //cyan
  _svColFG.push_back("\x1b[33m"); //yellow
//  _svColFG.push_back("\x1b[37m"); //white //
  _svColFG.push_back("\x1b[39m"); //white //ubuntuでは39がwhite, cygwinでは39/37が同じ
  _svColFG.push_back("\x1b[30m"); //black
  _svColFG.push_back("\x1b[37m"); //gray

  _svColBG.push_back("\x1b[49m"); //default
  _svColBG.push_back("\x1b[41m"); //red
  _svColBG.push_back("\x1b[42m"); //green
  _svColBG.push_back("\x1b[44m"); //blue
  _svColBG.push_back("\x1b[45m"); //magenta
  _svColBG.push_back("\x1b[46m"); //cyan
  _svColBG.push_back("\x1b[43m"); //yellow
  _svColBG.push_back("\x1b[47m"); //white  //linuxは対応してない
  _svColBG.push_back("\x1b[40m"); //black
  _svColBG.push_back("\x1b[47m"); //gray

  SetColor(eDefault, eDefault);
}


#endif

void ColorCout::SetColor(EColor eForeGround, EColor eBackGround) {


  _eForeGround = eForeGround;
  _eBackGround = eBackGround;
}

boost::iostreams::stream<ColorCout> &ColorCout::GetInstance() {
  static ColorCout sink;
  static boost::iostreams::stream<ColorCout> cc(sink);
  return cc;
}
