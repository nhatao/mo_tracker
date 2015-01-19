#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "Coordinates.h"

//連結座標系
//メモリ管理のトラブルを避けるため，boost::shared_ptr<CCascadedCoords>の形でしか作成できないようにした．
//GetVec,GetMatはワールド座標を返す
//GetLocalVec, GetLocalMatはローカル座標を返す

class ICascadedObjects {
public:
  virtual void NotifyUpdate() const = 0;
};

class CCascadedCoords :
  public CCoordinates3D, public ICascadedObjects
{
public:
  virtual ~CCascadedCoords(void);

  static boost::shared_ptr<CCascadedCoords> MakeCasCoords(double x, double y, double z, double yaw, double pitch, double roll, const std::string &rsName = "", boost::shared_ptr<const CCascadedCoords> pParent = GetWorldAnchor());
  static boost::shared_ptr<CCascadedCoords> MakeCasCoords(const CCoordinates3D &co, const std::string &rsName = "", boost::shared_ptr<const CCascadedCoords> pParent = GetWorldAnchor());

  virtual boost::shared_ptr<const CCascadedCoords> GetParent() const {return _pParent;}
  void SetParent(boost::shared_ptr<const CCascadedCoords> pParent);

  virtual void SetRot(const BoostMat& rot);
//  virtual void SetPos(const BoostVec& pos); //名前が被って使えない状態
  virtual void SetRPYAngle(double yaw, double pitch=0, double roll=0);
  virtual void SetQuartenion(const boost::math::quaternion<double>& q);
  virtual void SetPos(double x, double y, double z);
  virtual void Copy(const CCoordinates &rTarget);
//  virtual void Normalize();
  virtual void TransformLocal(const CCoordinates& coord);
  virtual void TransformWorld(const CCoordinates& coord);

  //この座標系で表現されるvec をワールド座標系の表現に変換する
  template <class T1, class T2>
  void TransformVector (const T1& vec, T2& result) const  { 
    Update();
    result = boost::numeric::ublas::prod(_rot,vec) + _pos;
  }
  //この座標系で表現されるvec をワールド座標系の表現に変換する
  template <class T1>
  void TransformVector (T1& vec) const {
    Update();
    vec = boost::numeric::ublas::prod(_rot,vec) + _pos;
  }
  //ワールド座標系におけるvec をローカル座標系の表現に逆変換する
  template <class T1, class T2>
  void InverseTransformVector (const T1& vec, T2& result) const { 
    Update();
    result = boost::numeric::ublas::prod(trans(_rot), vec) - prod(trans(_rot), _pos);
  }
  //ワールド座標系におけるvec をローカル座標系の表現に逆変換する
  template <class T1>
  void InverseTransformVector (T1& vec) const  {
    Update();
    vec = boost::numeric::ublas::prod(trans(_rot), vec) - prod(trans(_rot), _pos);
  }
  template <class T1, class T2>
  void VectorLocalToGlobal (const T1& vec, T2& result) const  { 
    Update();
    TransformVector(vec, result);
  }
  template <class T1>
  void VectorLocalToGlobal (T1& vec) const {
    Update();
    TransformVector(vec);
  }
  template <class T1, class T2>
  void VectorGlobalToLocal (const T1& vec, T2& result) const { 
    Update();
    InverseTransformVector(vec, result);
  }
  template <class T1>
  void VectorGlobalToLocal (T1& vec) const  {
    Update();
    InverseTransformVector(vec);
  }

  virtual const BoostMat& GetRot() const;
  virtual const BoostVec& GetPos() const;
  virtual double GetX() const;
  virtual double GetY() const;
  virtual double GetZ() const;
  virtual double GetYaw() const;
  virtual double GetPitch() const;
  virtual double GetRoll() const;

  const CCoordinates3D& GetLocal() const {return _Local;}
  virtual const BoostMat& GetLocalRot() const {return GetLocal().GetRot();}
  virtual const BoostVec& GetLocalPos() const {return GetLocal().GetPos();}
  virtual double GetLocalX() const {return GetLocal().GetX();}
  virtual double GetLocalY() const {return GetLocal().GetY();}
  virtual double GetLocalZ() const {return GetLocal().GetZ();}
  virtual double GetLocalYaw()   const {return GetLocal().GetYaw();}
  virtual double GetLocalPitch() const {return GetLocal().GetPitch();}
  virtual double GetLocalRoll()  const {return GetLocal().GetRoll();}

  const std::string &GetName() const {return _sName;}
  void SetName(const std::string &sName) {_sName = sName;}

  virtual std::string GetStr() const;

  static boost::shared_ptr<const CCascadedCoords> GetWorldAnchor();

  bool Changed() const {return _bChanged;}

  void AddChild(boost::shared_ptr<ICascadedObjects> pChild) const;
  
protected:

  CCascadedCoords(double x, double y, double z, double yaw, double pitch, double roll, boost::shared_ptr<const CCascadedCoords> pParent, const std::string &rsName);
  CCascadedCoords(const CCoordinates3D &co, boost::shared_ptr<const CCascadedCoords> pParent, const std::string &rsName);

  boost::shared_ptr<const CCascadedCoords> _pParent;
  mutable bool _bChanged;
  std::string _sName;

  virtual void Update() const; //他のconst関数から呼び出すため
  void UpdateImpl();
  
  CCoordinates3D _Local;
  CCascadedCoords(const CCascadedCoords &rhs);
  CCascadedCoords& operator= (const CCascadedCoords &rhs);
  mutable std::vector<boost::weak_ptr<ICascadedObjects> > _vpDescendants;
  virtual void NotifyUpdate() const;
};


//ワールド座標のアンカー．最終的な親
class CCascadedCoordsAnchor:  public CCascadedCoords {

friend class CCascadedCoords;
public:

  CCascadedCoordsAnchor() : CCascadedCoords(0,0,0,0,0,0,boost::shared_ptr<CCascadedCoords>(),"world") {
    _bChanged = false;
  }
  ~CCascadedCoordsAnchor() {}

  const CCoordinates3D& GetLocal() const {return _Local;}

  const std::string &GetName() const {return _sName;}
  void SetName(const std::string &sName) {_sName = sName;}

//  virtual bool NeedUpdate() const {return false;}
  virtual boost::shared_ptr<const CCascadedCoords> GetParent() const {
    throw std::logic_error("no parent");
    return _pParent;
  }

  virtual void Update() const {}
};

