#pragma once
#include <sstream>
#include <exception>
#include <string>

class ESStreamException : public std::exception
{

public:

  ESStreamException() {
    _pss = new std::stringstream;
  };
  virtual ~ESStreamException() throw(){
    delete _pss;
  };

  //めんどくさいがそれもこれもstringstreamがコピーできないのが悪い
  ESStreamException(const ESStreamException& e) {
    _pss = new std::stringstream;
    (*_pss) << (e._pss)->str();
  };
  ESStreamException& operator=(const ESStreamException& e) {
    if(this == &e) return *this;   //自己代入
    _pss = new std::stringstream;
    (*_pss) << (e._pss)->str();
    return *this;
  };
  virtual const char * what() const throw(){
    //_pss->str()は一時オブジェクトを作ってしまうので、その内容をコピーする必要がある
    _str = _pss->str();
    return (_str.c_str());
  }

  template <typename T>
  ESStreamException &operator << (const T& str) {
    (*_pss) << str;
    return (*this);
  }

protected:
  mutable std::string _str;//what()の中で変更されるが、意味的にはconst
  std::stringstream* _pss;
};
