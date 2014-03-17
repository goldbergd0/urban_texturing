// Dan Goldberg
// PLY Property class

#ifndef PROPERTY_H_
#define PROPERTY_H_
#include <iostream>
#include <string>

template <typename T>
class Property{
  public:
    Property();
      : name_(""), type_(""), tag_("") numel_(0),data_(new T[0]){};
    virtual ~Property();

    void setData(unsigned int size, const T* data){
      numel_ = size;
      data_ = data;
    };
    void setName(const std::string& s){name_=s;};
    void setType(const std::string& s){type_=s;};
    void setDataTag(const std::string& s){tag_=s;};
  
    unsigned int getDataSize(){return numel_;};
    T getData(const unsigned int ind)const{return T[ind];};
    std::string getDataTag()const{return tag_;};

    bool writePropertyHeader(FILE* file)const;
    //bool writePropertyData(const std::string& fname)const;
  
  private:
    std::string name_;
    std::string type_;
    std::string tag_;
    unsigned int numel_;
    T* data_;
  
};

template <typename T> 
Property<T>::~Property(){
  delete[] data_;
}

template <typename T>
bool Property<T>::writePropertyHeader(FILE* file)const{
  if (!file) return false;
  fprintf(file, "property %s %s\n", type_.c_str(), name_.c_str());
  return true;
}

#endif
