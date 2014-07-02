// Dan Goldberg
// Camera class
/*
  Holds the projection matrix
  Can project
  Holds image for that camera
*/

#include "../aux/camera.h"

Camera::Camera()
  :mat_(){}

Camera::~Camera(){}

Eigen::Vector2f Camera::project(const Eigen::Vector3f& v)const{
  Eigen::Vector4f inh;
  inh(0) = v(0);
  inh(1) = v(1);
  inh(2) = v(2);
  inh(3) = 1;
  Eigen::Vector3f outh(mat_*inh);
  Eigen::Vector2f out;
  out(0) = outh(0)/outh(2);
  out(1) = outh(1)/outh(2);
  return out;
}

Eigen::Vector2f Camera::project(const std::vector<float>& v)const{
  Eigen::Vector3f in(v.data());
  return project(in);
}

