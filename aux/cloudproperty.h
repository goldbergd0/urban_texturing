// Dan Goldberg
// Cloud Information class

#ifndef CLOUDPROPERTY_H_
#define CLOUDPROPERTY_H_
#include <iostream>
#include <string>

template <class T>
class CloudProperty{
public:
  CloudProperty();
  virtual ~CloudProperty();
  
  
  
private:
  std::string name_;
  std::string type_;
  
}
#endif
