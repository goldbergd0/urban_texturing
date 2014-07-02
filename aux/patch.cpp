#include "patch.h"

Patch::Patch()
  : point_(0,0,0),
    pointInd_(0),
    normal_(0,0,0),
    nImgs_(0),
    inds_(0)
  {}

//Patch::Patch(const Eigen::Vector3d& p,const Eigen::Vector3d& n,const unsigned int& nIm,
//      const Eigen::Vector3i& inds)
//  : point_(p),normal_(n),nImgs_(nIm),inds_(inds)
//  {}
Patch::Patch(const Patch& p)
  : point_(p.getPoint()),
    pointInd_(p.getPointInd()),
    normal_(p.getNormal()),
    nImgs_(p.getNImages()),
    inds_(p.getInds())
  {}

Patch::~Patch(){}

Patch& Patch::operator= (const Patch& p){
  point_ = p.getPoint();
  pointInd_ = p.getPointInd();
  normal_ = p.getNormal();
  nImgs_ = p.getNImages();
  inds_ = p.getInds();
  return *this;
}
 
std::ostream& operator<<(std::ostream& out,const Patch& p){
  out << p.getPoint();
  return out;
}
bool operator==(const Patch& p1,const pcl::PointXYZ& p2){
  return ((p1.getPoint().x==p2.x) &&
          (p1.getPoint().y==p2.y) &&
          (p1.getPoint().z==p2.z));
}
bool operator==(const pcl::PointXYZ& p1,const Patch& p2){
  return ((p2.getPoint().x==p1.x) &&
          (p2.getPoint().y==p1.y) &&
          (p2.getPoint().z==p1.z));
}

bool operator!=(const Patch& p1,const pcl::PointXYZ& p2){
  return ((p1.getPoint().x!=p2.x) ||
          (p1.getPoint().y!=p2.y) ||
          (p1.getPoint().z!=p2.z));
}
bool operator!=(const pcl::PointXYZ& p1,const Patch& p2){
  return ((p2.getPoint().x!=p1.x) ||
          (p2.getPoint().y!=p1.y) ||
          (p2.getPoint().z!=p1.z));
}
