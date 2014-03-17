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

  virtual void setData(const T* data);
  virtual void setName(const std::string& s){name_=s;};
  virtual void setType(const std::string& s){type_=s;};
  
  virtual bool writePropertyHeader(const std::string& fname)const = 0;
  virtual bool writePropertyData(const std::string& fname)const = 0;
  
private:
  std::string name_;
  std::string type_;
  T* data_;
  
}
#endif
