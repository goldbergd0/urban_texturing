// Dan Goldberg
// Element class

#ifndef ELEMENT_H_
#define ELEMENT_H_
#include <iostream>
#include <string>
#include <list>

template <class T>
class Element{
public:
  Element();
  virtual ~Element();

  void setName(const std::string& s){name_=s;};
  
  
private:
  std::string elname_;
  std::list<std::string> prnames_;
  std::list<std::string> prtypes_;
  T* ellist_;
  int numpr_;
  size_t numel_;
  std::list<CloudProperty> properties_;
}
#endif
