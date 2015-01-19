#pragma once

#include <string>
#include <vector>
#include <map>
#include <exception>
#include <float.h>
#include <boost/variant.hpp>
#include <boost/type_traits.hpp> 
#include <boost/utility/enable_if.hpp>
#include <sstream>
#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/lexical_cast.hpp> 
#include "StreamException.h"

class CPropertiedObject
{
public:

  CPropertiedObject() {} //デフォルトコンストラクタ、自分自身が保持する_varEntityには触れない前提
  template <class T> CPropertiedObject(T* var) { //ポインタ渡しなら、記憶領域は外部にある
    _varEntity = var;
  }
  template <class T> CPropertiedObject(const T& var) { //参照渡しなら、記憶領域を内部に作る
    boost::shared_ptr<T> p(new T);
    *p = var;
    _PropertyEntity = p;    
    _varEntity = p.get();
  }
  CPropertiedObject(const CPropertiedObject& rhs) {
    _mapProperties = rhs._mapProperties;
    _varEntity = rhs._varEntity;
    _PropertyEntity = rhs._PropertyEntity;
  }
  virtual ~CPropertiedObject(void) {}

  typedef boost::variant<int, double, std::string, bool>  Variant;
  typedef boost::variant<int*, double*, std::string*, bool*>  pVariant;
  typedef std::map <std::string, boost::shared_ptr<CPropertiedObject> > POMap;

  CPropertiedObject& AddProperty(const std::string& rsKey, const CPropertiedObject& rPO) {
    boost::shared_ptr<CPropertiedObject> ptr(new CPropertiedObject(rPO));
    return AddProperty(rsKey, ptr);
  }
  CPropertiedObject& AddProperty(const std::string& rsKey, boost::shared_ptr<CPropertiedObject> pPO) {
    std::pair<POMap::iterator, bool> pRes = _mapProperties.insert(std::make_pair(rsKey, pPO));
    if (pRes.second) {
      return *(pRes.first->second.get());
    }
    else {
      return GetChildProperty(rsKey);
    }
  }
  size_t RemoveProperty(const std::string& rsKey) {
    return _mapProperties.erase(rsKey);
  }
  void ClearProperty() {
    _mapProperties.clear();
  }

  const CPropertiedObject& GetChildProperty (const std::string& rsKey) const {
    POMap::const_iterator it = _mapProperties.find(rsKey);
    if (it != _mapProperties.end()) {
      return *(it->second.get());
    }
    ESStreamException e;
    e << "No Such Key: " << rsKey;
    throw e;
  }
  CPropertiedObject& GetChildProperty (const std::string& rsKey) {
    POMap::iterator it = _mapProperties.find(rsKey);
    if (it != _mapProperties.end()) {
      return *(it->second.get());
    }
    ESStreamException e;
    e << "No Such Key: " << rsKey;
    throw e;
  }
  const POMap& GetChildMap() const {return _mapProperties;}

  template <class T> bool SetValue(const std::string& rsKey, const T& rValue) {
    CPropertiedObject& rPO = GetChildProperty(rsKey);
    AssignVisitor<T> v(&rValue);
    if (apply_visitor(v, rPO._varEntity)) return true;
//    return false;
    ESStreamException e;
    e << "Invalid Type: " << "Property=(" << rPO._varEntity.type().name() << ") / Argument=(" << typeid(T).name() << ")";
    throw e;
  }
  Variant GetValue(const std::string& rsKey) const{
    const CPropertiedObject &rPO = GetChildProperty(rsKey);
    GetInstanceVisitor visitor;
    return apply_visitor(visitor, rPO._varEntity);
  }

  //boost::getが面倒なので
  template <class T> 
  T GetCastedValue(const std::string& rsKey) const{
    return boost::get<T>(GetValue(rsKey));
  }

  const pVariant& GetEntity() const {return _varEntity;}
  pVariant& GetEntity() {return _varEntity;}

private:

  //こうしておかないと、string->doubleみたいな訳分からんキャストのせいでコンパイルできない or Variantの要素と全く同じ型じゃないとコンパイルできない(floatとかshortとか)
  template<class T1, class T2>
  static bool copy_if_convertible(T1 *pT1, const T2 *pT2, typename boost::enable_if< boost::is_convertible<T1, T2> >::type* =0 ) {
    *pT1 = static_cast<T1>(*pT2);
    return true;
  }
  template<class T1, class T2>
  static bool copy_if_convertible(T1 *pT1, const T2 *pT2, typename boost::disable_if< boost::is_convertible<T1, T2> >::type* =0 ) {
    //lexical_castを試す
    try {
      T1 res = boost::lexical_cast<T1>(*pT2);
      *pT1 = res;
    }
    catch (...) {return false;}
    return true;
  }
  //警告が出るので消してみる
  template<class T2>
  static bool copy_if_convertible(bool *pT1, const T2 *pT2, typename boost::enable_if< boost::is_convertible<bool, T2> >::type* =0 ) {
    *pT1 = ((*pT2) != 0);
    return true;
  }
  //ポインタから実体を返すvisitor
  struct GetInstanceVisitor : public boost::static_visitor<Variant>
  {
    GetInstanceVisitor() : boost::static_visitor<Variant>() {}
    template<typename T> Variant operator()(const T* t ) {Variant v(*t); return v;}
  };
  //型に互換性があれば代入し、ないなら代入しない
  template <class T> struct AssignVisitor : public boost::static_visitor<bool>
  {
    AssignVisitor(const T* pValue) : boost::static_visitor<bool>() {
      _pValue = pValue;
    }
    template<typename T2> bool operator()(T2* pTarget) {
      return copy_if_convertible(pTarget, _pValue);
    }
    const T* _pValue;
  };

  POMap _mapProperties;
  pVariant _varEntity;

  boost::any _PropertyEntity; //ここに実体を格納する
};
