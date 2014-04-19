#include "patch.h"

Patch::Patch(const Eigen::Vector3d& p,const Eigen::Vector3d& n,const unsigned int& nIm,
      const Eigen::Vector3i& top3)
    : point_(p),normal_(n),nImgs_(nIm),top3_(top3){}
 


