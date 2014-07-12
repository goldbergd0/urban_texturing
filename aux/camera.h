// Dan Goldberg
// Camera Class

#ifndef CAMERA_H_
#define CAMERA_H_

#include <vector>
#include <string>

#include <Eigen/Dense>

#include <pcl/point_types.h>

class Camera{
public:
  Camera();
  Camera(const Camera& c);
  virtual ~Camera();
  
  void setMat(const Eigen::MatrixXf& m){mat_=m;};
  void setFileName(const std::string& fname){fname_=fname;};
  
  Eigen::MatrixXf getMat()const{return mat_;};
  std::string getFileName()const{return fname_;};
  
  Eigen::Vector2f project(const Eigen::Vector3f& v)const;
  Eigen::Vector2f project(const std::vector<float>& v)const;
  Eigen::Vector2f project(const pcl::PointXYZ& p)const;

  Camera& operator= (const Camera& p);
private:
  Eigen::MatrixXf mat_;
  std::string fname_;
  // ImageType im_;
};

#endif
