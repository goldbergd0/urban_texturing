// Dan Goldberg
// Cloud Option Information class

#ifndef PATCH_H_
#define PATCH_H_
#include <Eigen/Dense>

class Patch{
public:
  Patch();
  Patch(const Eigen::Vector3d& p,const Eigen::Vector3d& n,const unsigned int& nIm,
        const Eigen::Vector3i& top3);
  virtual ~Patch();
  
  Eigen::Vector3d getPoint()const {return point_;};
  Eigen::Vector3d getNormal()const {return normal_;};
  unsigned int getNImages()const {return nImgs_;};
  Eigen::Vector3i getTop3()const {return top3_;};
  
private:
  Eigen::Vector3d point_;
  Eigen::Vector3d normal_;
  unsigned int nImgs_;
  Eigen::Vector3i top3_;
  
};
#endif
