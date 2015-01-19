#pragma once
#include <iosfwd>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <vector>
#include <string>

/*
usage: ccout->SetColor(ColorCout::eRed, ColorCout::eDefault); などとして前景色・背景色設定 あとはcoutをccoutに置き換えるだけ
Win/Linux/Cygwin 全対応．ただし以下の制限あり
- ubuntuでは背景色が次の行まで適用される？
- GrayはWinのみ対応
*/

class ColorCout : public boost::iostreams::sink{
public:

  enum EColor {
    eDefault = 0,
    eRed, eGreen, eBlue, eMagenta, eCyan, eYellow, eWhite, eBlack, eGray, eColorNum
  };

  ColorCout();
  ~ColorCout() {};

  std::streamsize write(const char* s, std::streamsize n);
  void SetColor(EColor eForeGround = eDefault, EColor eBackGround = eDefault);

  static boost::iostreams::stream<ColorCout> &GetInstance();
  
protected:

  //win
  std::vector<unsigned int> _vColFG;
  std::vector<unsigned int> _vColBG;

  //linux
  std::vector<std::string> _svColFG;
  std::vector<std::string> _svColBG;

  EColor _eForeGround;
  EColor _eBackGround;
};

#define ccout ColorCout::GetInstance()
