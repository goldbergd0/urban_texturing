// Dan Goldberg
// Cloud Option Information class

#ifndef PATCH_H_
#define PATCH_H_

#include <vector>

#include <pcl/point_types.h>

class Patch{
public:
  Patch();
//  Patch(const pcl::PointXYZ& p,const pcl::Normal& n,const unsigned int& nIm,
//        const std::vector<unsigned int>& inds);
  Patch(const Patch& p);
  virtual ~Patch();
  
  pcl::PointXYZ getPoint()const {return point_;};
  size_t getPointInd()const{return pointInd_;};
  pcl::Normal getNormal()const {return normal_;};
  int getNImages()const {return nImgs_;};
  std::vector<int> getInds()const {return inds_;};
  
  void setPoint(pcl::PointXYZ p) {point_=p;};
  void setPointInd(size_t i){pointInd_=i;};
  void setNormal(pcl::Normal n) {normal_=n;};
  void setNImages(int N) {nImgs_=N;};
  void setInds(std::vector<int> inds){inds_=inds;};
  
  Patch& operator= (const Patch& p);
  
  friend bool operator==(const Patch& p1,const pcl::PointXYZ& p2);
  friend bool operator==(const pcl::PointXYZ& p1,const Patch& p2);
  friend bool operator!=(const Patch& p1,const pcl::PointXYZ& p2);
  friend bool operator!=(const pcl::PointXYZ& p1,const Patch& p2);

private:
  pcl::PointXYZ point_;
  size_t pointInd_;
  pcl::Normal normal_;
  int nImgs_;
  std::vector<int> inds_;
};
#endif
