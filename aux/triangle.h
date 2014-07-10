// Dan Goldberg
// Triangle object
// Template class using suggestions from 
//http://www.bogotobogo.com/cplusplus/template_declaration_definition_header_implementation_file.php
#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include <vector>

#include "../aux/patch.h"

// Template Declaration
template<typename T>
class Triangle{
  public:
    Triangle();
    Triangle(const T& v0,const T& v1,const T& v2);
    ~Triangle();

    void setv0(const T& v){v0_=v;};
    void setv1(const T& v){v1_=v;};
    void setv2(const T& v){v2_=v;};
    void setv(const int& i,const T& v);
    T getv0()const{return v0_;};
    T getv1()const{return v1_;};
    T getv2()const{return v2_;};
    T getv(const int& i)const;
  
  private:
    T v0_;
    T v1_;
    T v2_;
};

// Template Definition
template<typename T>
Triangle<T>::Triangle()
  :v0_(),v1_(),v2_() {}

template<typename T>
Triangle<T>::Triangle(const T& v0,const T& v1,const T& v2)
  :v0_(v0),v1_(v1),v2_(v2){}

template<typename T>
Triangle<T>::~Triangle() {}

template<typename T>
void Triangle<T>::setv(const int& i,const T& v){
  switch (i){
    case 0:
      setv0(v);
      break;
    case 1:
      setv1(v);
      break;
    case 2:
      setv2(v);
      break;
    default:
      throw std::out_of_range ("Set vector: i is out of range");
      break;
  }
}

template<typename T>
T Triangle<T>::getv(const int& i)const{
  switch (i){
    case 0:
      return getv0();
      break;
    case 1:
      return getv1();
      break;
    case 2:
      return getv2();
      break;
    default:
      throw std::out_of_range ("Get vector: i is out of range");
      break;
  }
  return T();
}
#endif
