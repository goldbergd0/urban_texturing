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
  PatchInfo(const unsigned int& n);
  virtual ~PatchInfo();
  
  void setPatches(const Patch* patches){patches_=patches;};
  Patch findPatch(const Eigen::Vector3d& point)const;
  
private:
  unsigned int nVerts_;
  Patch* patches_;
  
};
#endif
