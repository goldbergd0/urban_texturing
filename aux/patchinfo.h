// Dan Goldberg
// Cloud Option Information class

#ifndef PATCHINFO_H_
#define PATCHINFO_H_
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "../aux/patch.h"

class PatchInfo{
public:
  PatchInfo();
  virtual ~PatchInfo();
  
  Patch findPatch(const Eigen::Vector3d& point)const;
  
private:
  unsigned int nVerts_;
  Patch* patches_;
  
}
#endif
