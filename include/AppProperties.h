#pragma once
#include <iomanip>
#include <boost/variant.hpp>
#include <map>
#include <boost/typeof/typeof.hpp>
#include <boost/noncopyable.hpp>
#include "NHUtil.h"

class CProperties {
public:

  CProperties(const std::string &sName) {
    _sName = sName;
  }
  ~CProperties(){}

  void SetName(const std::string &sName) {
    _sName = sName;
  }
  virtual void ReadFromIniFile(const std::string &rsFileName) {
    for (auto it=_mProperties.begin(); it!=_mProperties.end(); ++it) {
      boost::apply_visitor(Reader(_sName, it->first, rsFileName), it->second);
    }
  }
  //既存のファイルは消して新しく作る
  virtual void WriteToIniFile(const std::string &rsFileName) {
    std::ofstream ofs(rsFileName.c_str());
    WriteToStream(ofs);
  }
  //追記したいときはこちらを使う
  void WriteToStream(std::ostream &ost) {
    ost << "[" << _sName << "]" << std::endl;
    for (auto it=_mProperties.begin(); it!=_mProperties.end(); ++it) {
      ost << it->first << "=";
      boost::apply_visitor(Writer(ost), it->second);
      ost << std::endl;
    }
  }

  void RemoveProperty(const std::string &sKey) {
    _mProperties.erase(sKey);
  }
  template <typename T>
  void AddProperty(const std::string &sKey, T *p) {
    if (_mProperties.find(sKey) != _mProperties.end()) {
      std::cout << "Warning: " << sKey << " Already Registered." << std::endl;
    }
    _mProperties[sKey] = p;
  }

protected:
  struct Writer : boost::static_visitor<void>
  {
    Writer(std::ostream &ost) : _ost(ost) {}
    template<typename T>
    void operator()( T& t ) const {     
      _ost << "\"" << *t << "\"";
//      _ost << *t;
    }
    void operator()( std::string*& t ) const {    
      _ost << "\"" << *t << "\"";
    }
    void operator()( double*& t ) const {     
      _ost << std::setprecision(16) << std::setiosflags(std::ios::fixed);
      _ost << "\"" << *t << "\"";
    }
    std::ostream& _ost;
  };

  struct Reader : boost::static_visitor<void>
  {
    Reader(const std::string &sName, const std::string &sKey, const std::string &sFileName)
      : _sName(sName), _sKey(sKey), _sFileName(sFileName) {}
    template<typename T>
    void operator()( T& t ) const {     
      char sBuf[2048];
      GetPrivateProfileString( _sName.c_str(),  _sKey.c_str(), 
        boost::lexical_cast<std::string>(*t).c_str(), sBuf, 2048, _sFileName.c_str());
      *t = boost::lexical_cast<BOOST_TYPEOF(*t)>(sBuf); 
    }
    void operator()( std::string*& t ) const {    
      char sBuf[2048];
      GetPrivateProfileString( _sName.c_str(),  _sKey.c_str(), 
        t->c_str(), sBuf, 2048, _sFileName.c_str());
      *t = sBuf;
    }

    std::string _sName;
    std::string _sKey;
    std::string _sFileName;

  };

  std::string _sName;
  std::map<std::string, boost::variant<bool*, int*, double*, std::string*> > _mProperties;


};