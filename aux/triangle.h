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
    T getv0()const{return v0_;};
    T getv1()const{return v1_;};
    T getv2()const{return v2_;};
  
  private:
    T v0_;
    T v1_;
    T v2_;
};

// Template Definition
template<typename T>
Triangle<T>::Triangle()
  :v0_(NULL),v1_(NULL),v2_(NULL) {}

template<typename T>
Triangle<T>::Triangle(const T& v0,const T& v1,const T& v2)
  :v0_(v0),v1_(v1),v2_(v2){}

template<typename T>
Triangle<T>::~Triangle() {}

#endif
