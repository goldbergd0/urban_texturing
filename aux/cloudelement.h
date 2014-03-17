// Dan Goldberg
// Cloud Information class

#ifndef CLOUDELEMENT_H_
#define CLOUDELEMENT_H_
#include <iostream>
#include <string>
#include <list>

template <class T>
class CloudElement{
public:
  CloudElement();
  virtual ~CloudElement();

  void setName(const std::string& s){name_=s;};
  void addProperty(const CloudProperty& p){properties_.push_front(p);};
  CloudProperty popProperty();

  
  
  std::string strHeader()const;

  
private:
  std::string name_;
  int nProperties_;
  std::list<CloudProperty> properties_;
}
#endif
