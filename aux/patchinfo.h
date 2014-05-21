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
 
  void addPatch(const Patch p,unsigned int i){patches_[i]=p;};
  void setPatches(const std::vector<Patch> patches){patches_=patches;};
  Patch findPatch(const Eigen::Vector3d& point)const;
  
private:
  unsigned int nVerts_;
  std::vector<Patch> patches_;
  
};
#endif
