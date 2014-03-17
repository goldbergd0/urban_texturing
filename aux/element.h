// Dan Goldberg
// Element class

#ifndef ELEMENT_H_
#define ELEMENT_H_
#include <iostream>
#include <string>
#include <list>
#include "property.h"

class Element{
public:
  Element();
  virtual ~Element();

  void setName(const std::string& s){name_=s;};
  void addProperty(const Property& p);
  Property popProperty();

  bool writeHeader(FILE* file)const;
  bool writeData(FILE* file)const;
  
private:
  std::string elname_;
  std::list<Property> properties_;
  int numpr_;
  int numel_;
}
#endif
