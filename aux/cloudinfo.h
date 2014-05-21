// Dan Goldberg
// Cloud Information class

#ifndef CLOUDINFO_H_
#define CLOUDINFO_H_
#include <iostream>
#include <vector>

class CloudInfo{
public:
  CloudInfo();
  virtual ~CloudInfo();
  
  
private:
  std::vector<float> verts_;
  unsigned long nVerts_;
  
}
#endif
