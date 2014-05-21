// Dan Goldberg
// Cloud Option Information class

#ifndef PATCH_H_
#define PATCH_H_

#include <vector>

#include <pcl/point_types.h>

class Patch{
public:
  Patch();
  Patch(const pcl::PointXYZ& p,const pcl::Normal& n,const unsigned int& nIm,
        const std::vector<unsigned int>& top3);
  Patch(const Patch& p);
  virtual ~Patch();
  
  pcl::PointXYZ getPoint()const {return point_;};
  pcl::Normal getNormal()const {return normal_;};
  unsigned int getNImages()const {return nImgs_;};
  std::vector<unsigned int> getTop3()const {return top3_;};
  
  Patch& operator= (const Patch& p);
  
  friend bool operator==(Patch& p1, pcl::PointXYZ p2);
  friend bool operator==(pcl::PointXYZ p1, Patch& p2);
  friend bool operator!=(Patch& p1, pcl::PointXYZ p2);
  friend bool operator!=(pcl::PointXYZ p1, Patch& p2);

private:
  pcl::PointXYZ point_;
  pcl::Normal normal_;
  unsigned int nImgs_;
  std::vector<unsigned int> top3_;
};
#endif
